"""
================================================================================
Project:    River Horizon — Autonomous Drone Fleet Management System
File:       safety/geofence.py
Purpose:    Geofence enforcement. Monitors the drone's GPS position against a
            configured cylindrical boundary (radius + altitude ceiling) and
            triggers RTH immediately on any breach. The geofence is ALWAYS
            active — it cannot be disabled at runtime.
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

from core.constants import (
    EARTH_RADIUS_M,
    GEOFENCE_CHECK_INTERVAL_S,
    FaultReason,
    SystemStatus,
)

logger = logging.getLogger(__name__)


class GeofenceMonitor:
    """
    Cylindrical geofence enforcer.

    Continuously polls the vehicle's GPS position and altitude. If the drone
    exits the defined cylinder (horizontal radius OR altitude ceiling), a
    GEOFENCE_BREACH fault is immediately reported to the fault manager, which
    triggers RTH.

    The home position is set on first GPS fix after initialisation.
    """

    def __init__(
        self,
        drone_id: str,
        radius_m: float,
        max_altitude_m: float,
        fault_manager,  # FaultManager — forward reference
    ) -> None:
        """
        Initialise the geofence monitor.

        Args:
            drone_id:       Drone identifier for logging.
            radius_m:       Maximum allowed horizontal distance from home (metres).
            max_altitude_m: Maximum allowed altitude above home (metres).
            fault_manager:  Central fault manager to report breaches to.
        """
        self._drone_id = drone_id
        self._radius_m = radius_m
        self._max_altitude_m = max_altitude_m
        self._fault_manager = fault_manager

        self._home_lat: Optional[float] = None
        self._home_lon: Optional[float] = None
        self._home_alt_m: Optional[float] = None
        self._home_set: bool = False

        self._breach_active: bool = False
        self._running: bool = False
        self._status: SystemStatus = SystemStatus.INITIALIZING

        # Bridge is injected after initialise() via set_bridge()
        self._bridge = None

    async def initialise(self) -> None:
        """
        Prepare the geofence monitor. Bridge must be set before run() is called.
        """
        self._status = SystemStatus.OK
        logger.info(
            "[%s] Geofence initialised: radius=%.0fm, ceiling=%.0fm.",
            self._drone_id, self._radius_m, self._max_altitude_m,
        )

    def set_bridge(self, bridge) -> None:
        """
        Inject the MAVLink bridge after it has been connected.

        Args:
            bridge: Active MAVLinkBridge instance.
        """
        self._bridge = bridge

    def set_home(self, latitude: float, longitude: float, altitude_m: float) -> None:
        """
        Set the geofence home/origin point.

        Args:
            latitude:   Home latitude in decimal degrees.
            longitude:  Home longitude in decimal degrees.
            altitude_m: Home altitude in metres (absolute).
        """
        self._home_lat = latitude
        self._home_lon = longitude
        self._home_alt_m = altitude_m
        self._home_set = True
        logger.info(
            "[%s] Geofence home set: (%.6f, %.6f, %.1fm).",
            self._drone_id, latitude, longitude, altitude_m,
        )

    # ------------------------------------------------------------------
    # Position validation (used by FlightController before issuing commands)
    # ------------------------------------------------------------------

    async def is_position_allowed(
        self,
        latitude: float,
        longitude: float,
        altitude_m: float,
    ) -> bool:
        """
        Check whether a target position is within the geofence.

        Args:
            latitude:   Target latitude in decimal degrees.
            longitude:  Target longitude in decimal degrees.
            altitude_m: Target altitude in metres (relative to home).

        Returns:
            True if the position is inside the geofence, False otherwise.
        """
        if not self._home_set:
            # No home set yet — allow all positions (pre-flight)
            return True

        dist = self._haversine_distance(
            self._home_lat, self._home_lon, latitude, longitude
        )
        if dist > self._radius_m:
            logger.warning(
                "[%s] Position (%.6f, %.6f) is %.1fm from home — outside geofence (%.0fm).",
                self._drone_id, latitude, longitude, dist, self._radius_m,
            )
            return False

        if altitude_m > self._max_altitude_m:
            logger.warning(
                "[%s] Altitude %.1fm exceeds geofence ceiling %.1fm.",
                self._drone_id, altitude_m, self._max_altitude_m,
            )
            return False

        return True

    # ------------------------------------------------------------------
    # Background enforcement loop
    # ------------------------------------------------------------------

    async def run(self) -> None:
        """
        Continuously monitor the drone's position against the geofence.
        Reports a fault immediately on any breach. Runs until cancelled.
        """
        self._running = True
        logger.info("[%s] Geofence enforcement loop started.", self._drone_id)

        while self._running:
            try:
                await self._check_position()
            except asyncio.CancelledError:
                break
            except Exception as exc:  # pylint: disable=broad-except
                logger.error("[%s] Geofence check error: %s", self._drone_id, exc)

            await asyncio.sleep(GEOFENCE_CHECK_INTERVAL_S)

        logger.info("[%s] Geofence enforcement loop stopped.", self._drone_id)

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def status(self) -> SystemStatus:
        """Current health status of the geofence monitor."""
        return self._status

    @property
    def is_breach_active(self) -> bool:
        """True if a geofence breach is currently active."""
        return self._breach_active

    @property
    def home_position(self) -> Optional[Tuple[float, float, float]]:
        """Home position as (lat, lon, alt_m) or None if not set."""
        if not self._home_set:
            return None
        return (self._home_lat, self._home_lon, self._home_alt_m)

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    async def _check_position(self) -> None:
        """
        Read the current vehicle position and check it against the geofence.
        Reports a fault if a breach is detected.
        """
        if self._bridge is None or not self._home_set:
            return

        state = await self._bridge.vehicle_state.snapshot()
        lat = state["latitude"]
        lon = state["longitude"]
        alt = state["relative_altitude_m"]

        # Skip check if we have no valid GPS fix
        if state["gps_fix"] < 2 or (lat == 0.0 and lon == 0.0):
            return

        dist = self._haversine_distance(self._home_lat, self._home_lon, lat, lon)
        altitude_breach = alt > self._max_altitude_m
        radius_breach = dist > self._radius_m

        if radius_breach or altitude_breach:
            if not self._breach_active:
                self._breach_active = True
                self._status = SystemStatus.CRITICAL
                reason = (
                    f"radius breach (dist={dist:.1f}m > {self._radius_m:.0f}m)"
                    if radius_breach
                    else f"altitude breach (alt={alt:.1f}m > {self._max_altitude_m:.0f}m)"
                )
                logger.critical(
                    "[%s] GEOFENCE BREACH: %s — triggering RTH.", self._drone_id, reason
                )
                await self._fault_manager.report_fault(
                    FaultReason.GEOFENCE_BREACH,
                    f"Geofence breach: {reason}",
                )
        else:
            if self._breach_active:
                self._breach_active = False
                self._status = SystemStatus.OK
                logger.info("[%s] Geofence breach cleared.", self._drone_id)

    @staticmethod
    def _haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """
        Calculate the great-circle distance between two GPS coordinates.

        Args:
            lat1, lon1: Origin in decimal degrees.
            lat2, lon2: Destination in decimal degrees.

        Returns:
            Distance in metres.
        """
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)
        a = (math.sin(dphi / 2) ** 2
             + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2)
        return 2 * EARTH_RADIUS_M * math.asin(math.sqrt(a))
