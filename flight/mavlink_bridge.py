"""
================================================================================
Project:    River Horizon — Autonomous Drone Fleet Management System
File:       flight/mavlink_bridge.py
Purpose:    Async MAVLink bridge. Manages the serial/UDP connection to the
            flight controller (Pixhawk / BetaFlight), dispatches incoming
            messages to registered handlers, and provides a thread-safe
            command interface for all flight subsystems.
Author:     [Author Placeholder]
Version:    1.0.0
Date:       2026-05-25
Part of:    River Song AI Ecosystem (riversongai.com)
================================================================================
"""

import asyncio
import logging
import time
from collections import defaultdict
from typing import Any, Callable, Dict, List, Optional

try:
    from pymavlink import mavutil
    from pymavlink.dialects.v20 import ardupilotmega as mavlink2
    MAVLINK_AVAILABLE = True
except ImportError:  # pragma: no cover
    MAVLINK_AVAILABLE = False

from core.constants import (
    MAVLINK_SYSTEM_ID,
    MAVLINK_COMPONENT_ID,
    MAVLINK_HEARTBEAT_TIMEOUT_S,
    HEARTBEAT_INTERVAL_S,
    ConnectionState,
    SystemStatus,
)

logger = logging.getLogger(__name__)


class MAVLinkBridgeError(Exception):
    """Raised when the MAVLink bridge encounters an unrecoverable error."""


class VehicleState:
    """
    Thread-safe snapshot of the latest vehicle telemetry received over MAVLink.
    Updated by the bridge receive loop; read by all other subsystems.
    """

    def __init__(self) -> None:
        """Initialise all state fields to safe defaults."""
        self.latitude: float        = 0.0
        self.longitude: float       = 0.0
        self.altitude_m: float      = 0.0
        self.relative_altitude_m: float = 0.0
        self.heading_deg: float     = 0.0
        self.speed_ms: float        = 0.0
        self.battery_voltage: float = 0.0
        self.battery_pct: float     = 100.0
        self.flight_mode: str       = "UNKNOWN"
        self.armed: bool            = False
        self.gps_fix: int           = 0
        self.gps_sats: int          = 0
        self.system_status: str     = SystemStatus.INITIALIZING
        self.last_heartbeat: float  = 0.0
        self._lock: asyncio.Lock    = asyncio.Lock()

    async def update(self, **kwargs: Any) -> None:
        """
        Atomically update one or more state fields.

        Args:
            **kwargs: Field names and their new values.
        """
        async with self._lock:
            for key, value in kwargs.items():
                if hasattr(self, key):
                    setattr(self, key, value)
                else:
                    logger.warning("VehicleState: unknown field '%s'", key)

    async def snapshot(self) -> Dict[str, Any]:
        """
        Return a consistent copy of all state fields.

        Returns:
            Dictionary of all state fields and their current values.
        """
        async with self._lock:
            return {
                "latitude":             self.latitude,
                "longitude":            self.longitude,
                "altitude_m":           self.altitude_m,
                "relative_altitude_m":  self.relative_altitude_m,
                "heading_deg":          self.heading_deg,
                "speed_ms":             self.speed_ms,
                "battery_voltage":      self.battery_voltage,
                "battery_pct":          self.battery_pct,
                "flight_mode":          self.flight_mode,
                "armed":                self.armed,
                "gps_fix":              self.gps_fix,
                "gps_sats":             self.gps_sats,
                "system_status":        self.system_status,
                "last_heartbeat":       self.last_heartbeat,
            }


class MAVLinkBridge:
    """
    Async MAVLink bridge between River Horizon and the flight controller.

    Responsibilities:
    - Establish and maintain the MAVLink serial/UDP connection.
    - Run a non-blocking receive loop that dispatches messages to handlers.
    - Send heartbeats to keep the GCS link alive.
    - Expose send_command() for all outbound MAVLink commands.
    - Expose vehicle_state for real-time telemetry reads.
    """

    def __init__(
        self,
        connection_string: str,
        baud_rate: int,
        drone_id: str,
        system_id: int = MAVLINK_SYSTEM_ID,
        component_id: int = MAVLINK_COMPONENT_ID,
    ) -> None:
        """
        Initialise the bridge (does not connect yet — call connect()).

        Args:
            connection_string: Serial port or UDP address (e.g. '/dev/ttyAMA0',
                               'udp:127.0.0.1:14550').
            baud_rate:         Serial baud rate (ignored for UDP connections).
            drone_id:          Human-readable drone identifier for logging.
            system_id:         MAVLink GCS system ID.
            component_id:      MAVLink GCS component ID.
        """
        self._connection_string = connection_string
        self._baud_rate = baud_rate
        self._drone_id = drone_id
        self._system_id = system_id
        self._component_id = component_id

        self._connection: Optional[Any] = None
        self._state: ConnectionState = ConnectionState.DISCONNECTED
        self._handlers: Dict[str, List[Callable]] = defaultdict(list)
        self._running: bool = False

        self.vehicle_state = VehicleState()

        # Register built-in message handlers
        self._register_builtin_handlers()

    # ------------------------------------------------------------------
    # Connection lifecycle
    # ------------------------------------------------------------------

    async def connect(self) -> None:
        """
        Open the MAVLink connection and wait for the first heartbeat.

        Raises:
            MAVLinkBridgeError: If the connection cannot be established or
                                no heartbeat is received within the timeout.
        """
        if not MAVLINK_AVAILABLE:
            logger.warning("pymavlink not installed — running in simulation mode.")
            self._state = ConnectionState.CONNECTED
            return

        logger.info("[%s] Connecting to flight controller: %s",
                    self._drone_id, self._connection_string)
        try:
            self._connection = mavutil.mavlink_connection(
                self._connection_string,
                baud=self._baud_rate,
                source_system=self._system_id,
                source_component=self._component_id,
            )
        except Exception as exc:
            raise MAVLinkBridgeError(
                f"Failed to open MAVLink connection '{self._connection_string}': {exc}"
            ) from exc

        # Wait for heartbeat with timeout
        logger.info("[%s] Waiting for heartbeat...", self._drone_id)
        loop = asyncio.get_running_loop()
        try:
            await asyncio.wait_for(
                loop.run_in_executor(None, self._connection.wait_heartbeat),
                timeout=MAVLINK_HEARTBEAT_TIMEOUT_S * 3,
            )
        except asyncio.TimeoutError as exc:
            raise MAVLinkBridgeError(
                f"No heartbeat received from flight controller within "
                f"{MAVLINK_HEARTBEAT_TIMEOUT_S * 3:.0f}s"
            ) from exc

        self._state = ConnectionState.CONNECTED
        await self.vehicle_state.update(last_heartbeat=time.monotonic())
        logger.info("[%s] MAVLink connection established.", self._drone_id)

    async def disconnect(self) -> None:
        """Close the MAVLink connection cleanly."""
        self._running = False
        if self._connection and MAVLINK_AVAILABLE:
            try:
                self._connection.close()
            except Exception as exc:  # pylint: disable=broad-except
                logger.warning("[%s] Error closing MAVLink connection: %s", self._drone_id, exc)
        self._state = ConnectionState.DISCONNECTED
        logger.info("[%s] MAVLink connection closed.", self._drone_id)

    # ------------------------------------------------------------------
    # Background run loop
    # ------------------------------------------------------------------

    async def run(self) -> None:
        """
        Main receive loop. Reads incoming MAVLink messages and dispatches
        them to registered handlers. Also sends periodic heartbeats.
        Runs until disconnect() is called.
        """
        self._running = True
        last_hb = 0.0
        loop = asyncio.get_running_loop()

        logger.info("[%s] MAVLink receive loop started.", self._drone_id)

        while self._running:
            now = time.monotonic()

            # Send GCS heartbeat
            if now - last_hb >= HEARTBEAT_INTERVAL_S:
                await self._send_heartbeat()
                last_hb = now

            # Check for link timeout
            snap = await self.vehicle_state.snapshot()
            if (snap["last_heartbeat"] > 0 and
                    now - snap["last_heartbeat"] > MAVLINK_HEARTBEAT_TIMEOUT_S):
                if self._state != ConnectionState.DISCONNECTED:
                    logger.warning("[%s] Heartbeat timeout — link lost.", self._drone_id)
                    self._state = ConnectionState.DISCONNECTED

            # Non-blocking message receive
            if self._connection and MAVLINK_AVAILABLE:
                try:
                    msg = await loop.run_in_executor(
                        None, lambda: self._connection.recv_match(blocking=False)
                    )
                    if msg:
                        await self._dispatch(msg)
                except Exception as exc:  # pylint: disable=broad-except
                    logger.error("[%s] MAVLink receive error: %s", self._drone_id, exc)

            await asyncio.sleep(0.01)  # 100 Hz poll ceiling

    # ------------------------------------------------------------------
    # Command interface
    # ------------------------------------------------------------------

    async def send_command_long(
        self,
        command: int,
        param1: float = 0,
        param2: float = 0,
        param3: float = 0,
        param4: float = 0,
        param5: float = 0,
        param6: float = 0,
        param7: float = 0,
        confirmation: int = 0,
    ) -> bool:
        """
        Send a MAVLink COMMAND_LONG message to the vehicle.

        Args:
            command:      MAVLink command ID (e.g. mavutil.mavlink.MAV_CMD_*).
            param1-7:     Command parameters (meaning depends on command).
            confirmation: Confirmation counter (0 for first attempt).

        Returns:
            True if the command was sent successfully, False otherwise.
        """
        if not self._connection or not MAVLINK_AVAILABLE:
            logger.debug("[%s] Simulation: COMMAND_LONG cmd=%d", self._drone_id, command)
            return True
        try:
            self._connection.mav.command_long_send(
                self._connection.target_system,
                self._connection.target_component,
                command,
                confirmation,
                param1, param2, param3, param4, param5, param6, param7,
            )
            return True
        except Exception as exc:  # pylint: disable=broad-except
            logger.error("[%s] Failed to send COMMAND_LONG %d: %s", self._drone_id, command, exc)
            return False

    async def set_mode(self, mode_name: str) -> bool:
        """
        Set the vehicle flight mode by name.

        Args:
            mode_name: Mode string (e.g. 'GUIDED', 'RTL', 'LAND').

        Returns:
            True if the mode change command was sent successfully.
        """
        if not self._connection or not MAVLINK_AVAILABLE:
            logger.debug("[%s] Simulation: set_mode(%s)", self._drone_id, mode_name)
            await self.vehicle_state.update(flight_mode=mode_name)
            return True
        try:
            mode_id = self._connection.mode_mapping().get(mode_name)
            if mode_id is None:
                logger.error("[%s] Unknown flight mode: %s", self._drone_id, mode_name)
                return False
            self._connection.set_mode(mode_id)
            logger.info("[%s] Mode change requested: %s", self._drone_id, mode_name)
            return True
        except Exception as exc:  # pylint: disable=broad-except
            logger.error("[%s] Failed to set mode %s: %s", self._drone_id, mode_name, exc)
            return False

    async def arm(self, force: bool = False) -> bool:
        """
        Arm the vehicle motors.

        Args:
            force: If True, bypass pre-arm checks (use with extreme caution).

        Returns:
            True if the arm command was sent successfully.
        """
        param2 = 21196.0 if force else 0.0
        logger.info("[%s] Arming motors (force=%s).", self._drone_id, force)
        if not MAVLINK_AVAILABLE:
            await self.vehicle_state.update(armed=True)
            return True
        return await self.send_command_long(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1, param2
        )

    async def disarm(self) -> bool:
        """
        Disarm the vehicle motors.

        Returns:
            True if the disarm command was sent successfully.
        """
        logger.info("[%s] Disarming motors.", self._drone_id)
        if not MAVLINK_AVAILABLE:
            await self.vehicle_state.update(armed=False)
            return True
        return await self.send_command_long(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0
        )

    # ------------------------------------------------------------------
    # Handler registration
    # ------------------------------------------------------------------

    def register_handler(self, message_type: str, handler: Callable) -> None:
        """
        Register a callback for a specific MAVLink message type.

        Args:
            message_type: MAVLink message name (e.g. 'HEARTBEAT', 'GPS_RAW_INT').
            handler:      Async callable that accepts the MAVLink message object.
        """
        self._handlers[message_type].append(handler)
        logger.debug("[%s] Registered handler for %s", self._drone_id, message_type)

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def connection_state(self) -> ConnectionState:
        """Current connection state of the MAVLink link."""
        return self._state

    @property
    def is_connected(self) -> bool:
        """True if the MAVLink link is currently connected."""
        return self._state == ConnectionState.CONNECTED

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _register_builtin_handlers(self) -> None:
        """Register internal handlers for core MAVLink message types."""
        self.register_handler("HEARTBEAT",          self._on_heartbeat)
        self.register_handler("GLOBAL_POSITION_INT", self._on_global_position)
        self.register_handler("VFR_HUD",            self._on_vfr_hud)
        self.register_handler("SYS_STATUS",         self._on_sys_status)
        self.register_handler("GPS_RAW_INT",        self._on_gps_raw)
        self.register_handler("BATTERY_STATUS",     self._on_battery_status)

    async def _dispatch(self, msg: Any) -> None:
        """
        Dispatch a received MAVLink message to all registered handlers.

        Args:
            msg: The received MAVLink message object.
        """
        msg_type = msg.get_type()
        handlers = self._handlers.get(msg_type, [])
        for handler in handlers:
            try:
                await handler(msg)
            except Exception as exc:  # pylint: disable=broad-except
                logger.error("[%s] Handler error for %s: %s", self._drone_id, msg_type, exc)

    async def _send_heartbeat(self) -> None:
        """Send a GCS heartbeat to the flight controller."""
        if not self._connection or not MAVLINK_AVAILABLE:
            return
        try:
            self._connection.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0,
            )
        except Exception as exc:  # pylint: disable=broad-except
            logger.warning("[%s] Failed to send heartbeat: %s", self._drone_id, exc)

    # ------------------------------------------------------------------
    # Built-in message handlers
    # ------------------------------------------------------------------

    async def _on_heartbeat(self, msg: Any) -> None:
        """Handle incoming HEARTBEAT — update armed state and link timestamp."""
        armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) \
            if MAVLINK_AVAILABLE else False
        await self.vehicle_state.update(
            armed=armed,
            last_heartbeat=time.monotonic(),
        )
        if self._state != ConnectionState.CONNECTED:
            self._state = ConnectionState.CONNECTED
            logger.info("[%s] MAVLink link restored.", self._drone_id)

    async def _on_global_position(self, msg: Any) -> None:
        """Handle GLOBAL_POSITION_INT — update lat/lon/alt."""
        await self.vehicle_state.update(
            latitude=msg.lat / 1e7,
            longitude=msg.lon / 1e7,
            altitude_m=msg.alt / 1000.0,
            relative_altitude_m=msg.relative_alt / 1000.0,
            heading_deg=msg.hdg / 100.0 if msg.hdg != 65535 else 0.0,
        )

    async def _on_vfr_hud(self, msg: Any) -> None:
        """Handle VFR_HUD — update airspeed and heading."""
        await self.vehicle_state.update(
            speed_ms=msg.groundspeed,
            heading_deg=float(msg.heading),
        )

    async def _on_sys_status(self, msg: Any) -> None:
        """Handle SYS_STATUS — update battery voltage and percentage."""
        await self.vehicle_state.update(
            battery_voltage=msg.voltage_battery / 1000.0,
            battery_pct=msg.battery_remaining if msg.battery_remaining >= 0 else 100.0,
        )

    async def _on_gps_raw(self, msg: Any) -> None:
        """Handle GPS_RAW_INT — update GPS fix type and satellite count."""
        await self.vehicle_state.update(
            gps_fix=msg.fix_type,
            gps_sats=msg.satellites_visible,
        )

    async def _on_battery_status(self, msg: Any) -> None:
        """Handle BATTERY_STATUS — update battery percentage."""
        if msg.battery_remaining >= 0:
            await self.vehicle_state.update(battery_pct=float(msg.battery_remaining))
