"""
================================================================================
Project:    River Horizon — Autonomous Drone Fleet Management System
File:       flight/flight_controller.py
Purpose:    High-level flight controller. Provides arm/disarm, takeoff, land,
            and position-command primitives. Enforces geofence and altitude
            limits on every command before forwarding to the MAVLink bridge.
Author:     [Author Placeholder]
Version:    1.0.0
Date:       2026-05-25
Part of:    River Song AI Ecosystem (riversongai.com)
================================================================================
"""

import asyncio
import logging
import math
from typing import Optional, Tuple

from core.constants import FlightMode, FaultReason, EARTH_RADIUS_M
from flight.mavlink_bridge import MAVLinkBridge
from flight.mode_manager import ModeManager

logger = logging.getLogger(__name__)


class FlightCommandError(Exception):
    """Raised when a flight command is rejected due to safety or state constraints."""


class FlightController:
    """
    High-level flight command interface for a single drone unit.

    All commands pass through geofence and altitude validation before being
    forwarded to the MAVLink bridge. This is the primary interface used by
    the waypoint manager, web controller, and River Song voice commands.
    """

    def __init__(
        self,
        bridge: MAVLinkBridge,
        mode_manager: ModeManager,
        geofence,           # GeofenceMonitor — forward reference avoids circular import
        fault_manager,      # FaultManager
        max_altitude_m: float,
        max_speed_ms: float,
    ) -> None:
        """
        Initialise the flight controller.

        Args:
            bridge:          Active MAVLink bridge.
            mode_manager:    Mode manager for flight mode transitions.
            geofence:        Active geofence monitor for boundary checks.
            fault_manager:   Central fault manager for reporting violations.
            max_altitude_m:  Hard ceiling for all altitude commands.
            max_speed_ms:    Hard ceiling for all speed commands.
        """
        self._bridge = bridge
        self._mode_manager = mode_manager
        self._geofence = geofence
        self._fault_manager = fault_manager
        self._max_altitude_m = max_altitude_m
        self._max_speed_ms = max_speed_ms
        self._drone_id = bridge._drone_id

    async def initialise(self) -> None:
        """Perform any async initialisation required before accepting commands."""
        logger.info("[%s] FlightController initialised.", self._drone_id)

    # ------------------------------------------------------------------
    # Arm / Disarm
    # ------------------------------------------------------------------

    async def arm(self) -> bool:
        """
        Arm the vehicle after pre-arm safety checks.

        Returns:
            True if arming succeeded.

        Raises:
            FlightCommandError: If pre-arm checks fail.
        """
        state = await self._bridge.vehicle_state.snapshot()

        if state["gps_fix"] < 3:
            raise FlightCommandError(
                f"Pre-arm failed: GPS fix insufficient (fix={state['gps_fix']}, need ≥3)."
            )
        if state["battery_pct"] < 30.0:
            raise FlightCommandError(
                f"Pre-arm failed: Battery too low to arm ({state['battery_pct']:.0f}%)."
            )

        logger.info("[%s] Arming vehicle.", self._drone_id)
        return await self._bridge.arm()

    async def disarm(self) -> bool:
        """
        Disarm the vehicle. Only permitted when on the ground.

        Returns:
            True if disarming succeeded.

        Raises:
            FlightCommandError: If the vehicle is airborne.
        """
        state = await self._bridge.vehicle_state.snapshot()
        if state["relative_altitude_m"] > 0.5:
            raise FlightCommandError(
                "Cannot disarm while airborne "
                f"(altitude={state['relative_altitude_m']:.1f}m)."
            )
        logger.info("[%s] Disarming vehicle.", self._drone_id)
        return await self._bridge.disarm()

    # ------------------------------------------------------------------
    # Takeoff / Land
    # ------------------------------------------------------------------

    async def takeoff(self, altitude_m: float) -> bool:
        """
        Command the vehicle to take off to the specified altitude.

        Args:
            altitude_m: Target altitude in metres (relative to home).

        Returns:
            True if the takeoff command was accepted.

        Raises:
            FlightCommandError: If altitude exceeds limits or vehicle is not armed.
        """
        altitude_m = self._clamp_altitude(altitude_m)
        state = await self._bridge.vehicle_state.snapshot()

        if not state["armed"]:
            raise FlightCommandError("Cannot take off — vehicle is not armed.")

        logger.info("[%s] Takeoff to %.1fm.", self._drone_id, altitude_m)
        await self._mode_manager.request_mode(FlightMode.GUIDED)

        try:
            from pymavlink import mavutil
            return await self._bridge.send_command_long(
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                param7=altitude_m,
            )
        except ImportError:
            # Simulation mode
            logger.debug("[%s] Simulation: takeoff to %.1fm", self._drone_id, altitude_m)
            return True

    async def land(self) -> bool:
        """
        Command the vehicle to land at the current position.

        Returns:
            True if the land command was accepted.
        """
        logger.info("[%s] Landing at current position.", self._drone_id)
        return await self._mode_manager.request_mode(FlightMode.LAND, override_safety=True)

    # ------------------------------------------------------------------
    # Position commands
    # ------------------------------------------------------------------

    async def goto_position(
        self,
        latitude: float,
        longitude: float,
        altitude_m: float,
        speed_ms: Optional[float] = None,
    ) -> bool:
        """
        Command the vehicle to fly to a specific GPS coordinate.

        Args:
            latitude:   Target latitude in decimal degrees.
            longitude:  Target longitude in decimal degrees.
            altitude_m: Target altitude in metres (relative to home).
            speed_ms:   Optional cruise speed override in m/s.

        Returns:
            True if the command was accepted.

        Raises:
            FlightCommandError: If the target position violates geofence or
                                altitude limits.
        """
        altitude_m = self._clamp_altitude(altitude_m)

        # Geofence check on target position
        if not await self._geofence.is_position_allowed(latitude, longitude, altitude_m):
            await self._fault_manager.report_fault(
                FaultReason.GEOFENCE_BREACH,
                f"goto_position target ({latitude:.6f}, {longitude:.6f}) "
                f"is outside geofence.",
            )
            raise FlightCommandError(
                f"Target position ({latitude:.6f}, {longitude:.6f}) violates geofence."
            )

        if speed_ms is not None:
            speed_ms = min(speed_ms, self._max_speed_ms)
            await self._set_speed(speed_ms)

        await self._mode_manager.request_mode(FlightMode.GUIDED)

        try:
            from pymavlink import mavutil
            return await self._bridge.send_command_long(
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                param5=latitude,
                param6=longitude,
                param7=altitude_m,
            )
        except ImportError:
            logger.debug("[%s] Simulation: goto (%.6f, %.6f, %.1fm)",
                         self._drone_id, latitude, longitude, altitude_m)
            return True

    async def set_velocity(
        self,
        vx: float,
        vy: float,
        vz: float,
    ) -> bool:
        """
        Command velocity in the body frame (used for manual joystick control).

        Args:
            vx: Forward velocity in m/s (positive = forward).
            vy: Right velocity in m/s (positive = right).
            vz: Down velocity in m/s (positive = down).

        Returns:
            True if the command was sent.
        """
        # Clamp to max speed
        magnitude = math.sqrt(vx**2 + vy**2 + vz**2)
        if magnitude > self._max_speed_ms:
            scale = self._max_speed_ms / magnitude
            vx, vy, vz = vx * scale, vy * scale, vz * scale

        try:
            from pymavlink import mavutil
            # SET_POSITION_TARGET_LOCAL_NED with velocity mask
            if self._bridge._connection:
                self._bridge._connection.mav.set_position_target_local_ned_send(
                    0,  # time_boot_ms
                    self._bridge._connection.target_system,
                    self._bridge._connection.target_component,
                    mavutil.mavlink.MAV_FRAME_BODY_NED,
                    0b0000111111000111,  # velocity only mask
                    0, 0, 0,            # position (ignored)
                    vx, vy, vz,         # velocity
                    0, 0, 0,            # acceleration (ignored)
                    0, 0,               # yaw, yaw_rate (ignored)
                )
            return True
        except ImportError:
            logger.debug("[%s] Simulation: set_velocity(%.1f, %.1f, %.1f)", self._drone_id, vx, vy, vz)
            return True

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _clamp_altitude(self, altitude_m: float) -> float:
        """
        Clamp an altitude value to the configured maximum.

        Args:
            altitude_m: Requested altitude in metres.

        Returns:
            Clamped altitude, never exceeding max_altitude_m.
        """
        if altitude_m > self._max_altitude_m:
            logger.warning(
                "[%s] Altitude %.1fm clamped to max %.1fm.",
                self._drone_id, altitude_m, self._max_altitude_m,
            )
            return self._max_altitude_m
        return altitude_m

    async def _set_speed(self, speed_ms: float) -> None:
        """
        Send a speed override command to the vehicle.

        Args:
            speed_ms: Desired cruise speed in m/s.
        """
        try:
            from pymavlink import mavutil
            await self._bridge.send_command_long(
                mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                param1=1,           # 1 = ground speed
                param2=speed_ms,
                param3=-1,          # throttle (no change)
            )
        except ImportError:
            logger.debug("[%s] Simulation: set_speed(%.1f)", self._drone_id, speed_ms)
