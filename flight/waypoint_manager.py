"""
================================================================================
Project:    River Horizon — Autonomous Drone Fleet Management System
File:       flight/waypoint_manager.py
Purpose:    Waypoint mission management. Builds, uploads, and monitors
            MAVLink waypoint missions. Supports named waypoint aliases
            (e.g. "waypoint alpha") for River Song voice command integration.
Author:     [Author Placeholder]
Version:    1.0.0
Date:       2026-05-25
Part of:    River Song AI Ecosystem (riversongai.com)
================================================================================
"""

import asyncio
import logging
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional

from core.constants import EARTH_RADIUS_M
from flight.mavlink_bridge import MAVLinkBridge
from flight.flight_controller import FlightController

logger = logging.getLogger(__name__)


@dataclass
class Waypoint:
    """A single geographic waypoint with optional metadata."""
    name: str
    latitude: float
    longitude: float
    altitude_m: float
    loiter_time_s: float = 0.0
    speed_ms: Optional[float] = None
    action: str = "WAYPOINT"   # WAYPOINT | LOITER | LAND | TAKEOFF


@dataclass
class Mission:
    """An ordered sequence of waypoints forming a complete flight mission."""
    name: str
    waypoints: List[Waypoint] = field(default_factory=list)
    repeat: bool = False
    current_index: int = 0

    def add_waypoint(self, wp: Waypoint) -> None:
        """Append a waypoint to the mission."""
        self.waypoints.append(wp)

    @property
    def is_complete(self) -> bool:
        """True if all waypoints have been visited."""
        return self.current_index >= len(self.waypoints)

    @property
    def current_waypoint(self) -> Optional[Waypoint]:
        """The waypoint currently being navigated to, or None if complete."""
        if self.is_complete:
            return None
        return self.waypoints[self.current_index]


class WaypointManager:
    """
    Manages waypoint missions for a single drone unit.

    Supports:
    - Named waypoint registry (for River Song voice commands)
    - Sequential mission execution via FlightController.goto_position()
    - Arrival detection using haversine distance
    - Mission pause, resume, and abort
    """

    def __init__(
        self,
        bridge: MAVLinkBridge,
        flight_controller: FlightController,
        waypoint_radius_m: float = 2.0,
    ) -> None:
        """
        Initialise the waypoint manager.

        Args:
            bridge:             Active MAVLink bridge for position reads.
            flight_controller:  Flight controller for issuing goto commands.
            waypoint_radius_m:  Arrival radius — waypoint is considered reached
                                when the drone is within this distance.
        """
        self._bridge = bridge
        self._fc = flight_controller
        self._waypoint_radius_m = waypoint_radius_m
        self._drone_id = bridge._drone_id

        self._named_waypoints: Dict[str, Waypoint] = {}
        self._active_mission: Optional[Mission] = None
        self._mission_task: Optional[asyncio.Task] = None
        self._paused: bool = False
        self._lock = asyncio.Lock()

    # ------------------------------------------------------------------
    # Named waypoint registry
    # ------------------------------------------------------------------

    def register_waypoint(self, waypoint: Waypoint) -> None:
        """
        Register a named waypoint for use in voice commands and missions.

        Args:
            waypoint: The waypoint to register. Its name is used as the key.
        """
        key = waypoint.name.lower().strip()
        self._named_waypoints[key] = waypoint
        logger.info("[%s] Registered waypoint '%s' at (%.6f, %.6f, %.1fm).",
                    self._drone_id, waypoint.name,
                    waypoint.latitude, waypoint.longitude, waypoint.altitude_m)

    def get_waypoint(self, name: str) -> Optional[Waypoint]:
        """
        Retrieve a named waypoint by name (case-insensitive).

        Args:
            name: Waypoint name (e.g. 'alpha', 'home base').

        Returns:
            The Waypoint if found, otherwise None.
        """
        return self._named_waypoints.get(name.lower().strip())

    def list_waypoints(self) -> List[str]:
        """
        Return a list of all registered waypoint names.

        Returns:
            Sorted list of waypoint name strings.
        """
        return sorted(self._named_waypoints.keys())

    # ------------------------------------------------------------------
    # Mission control
    # ------------------------------------------------------------------

    async def start_mission(self, mission: Mission) -> None:
        """
        Begin executing a waypoint mission.

        Args:
            mission: The Mission to execute.

        Raises:
            RuntimeError: If a mission is already active.
        """
        async with self._lock:
            if self._mission_task and not self._mission_task.done():
                raise RuntimeError(
                    f"[{self._drone_id}] Cannot start mission '{mission.name}' — "
                    "a mission is already active. Abort it first."
                )
            self._active_mission = mission
            self._paused = False
            self._mission_task = asyncio.create_task(
                self._execute_mission(mission),
                name=f"mission_{mission.name}",
            )
            logger.info("[%s] Mission '%s' started (%d waypoints).",
                        self._drone_id, mission.name, len(mission.waypoints))

    async def goto_named_waypoint(self, name: str) -> bool:
        """
        Fly directly to a single named waypoint (no full mission).

        Args:
            name: Registered waypoint name.

        Returns:
            True if the goto command was accepted.

        Raises:
            KeyError: If the waypoint name is not registered.
        """
        wp = self.get_waypoint(name)
        if wp is None:
            raise KeyError(f"Waypoint '{name}' is not registered.")
        logger.info("[%s] Flying to named waypoint '%s'.", self._drone_id, name)
        return await self._fc.goto_position(
            wp.latitude, wp.longitude, wp.altitude_m, wp.speed_ms
        )

    async def pause_mission(self) -> None:
        """Pause the active mission at the current waypoint."""
        self._paused = True
        logger.info("[%s] Mission paused.", self._drone_id)

    async def resume_mission(self) -> None:
        """Resume a paused mission from the current waypoint."""
        self._paused = False
        logger.info("[%s] Mission resumed.", self._drone_id)

    async def abort_mission(self) -> None:
        """
        Abort the active mission and cancel the background task.
        The drone remains in its current position/mode.
        """
        async with self._lock:
            if self._mission_task and not self._mission_task.done():
                self._mission_task.cancel()
                try:
                    await self._mission_task
                except asyncio.CancelledError:
                    pass
            self._active_mission = None
            self._paused = False
            logger.info("[%s] Mission aborted.", self._drone_id)

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def is_mission_active(self) -> bool:
        """True if a mission is currently executing."""
        return (
            self._active_mission is not None
            and self._mission_task is not None
            and not self._mission_task.done()
        )

    @property
    def active_mission(self) -> Optional[Mission]:
        """The currently active Mission, or None."""
        return self._active_mission

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    async def _execute_mission(self, mission: Mission) -> None:
        """
        Internal coroutine that drives the drone through each waypoint.

        Args:
            mission: The mission to execute.
        """
        logger.info("[%s] Executing mission '%s'.", self._drone_id, mission.name)
        try:
            while not mission.is_complete:
                if self._paused:
                    await asyncio.sleep(0.5)
                    continue

                wp = mission.current_waypoint
                logger.info(
                    "[%s] Navigating to waypoint %d/%d: '%s' (%.6f, %.6f, %.1fm).",
                    self._drone_id,
                    mission.current_index + 1,
                    len(mission.waypoints),
                    wp.name, wp.latitude, wp.longitude, wp.altitude_m,
                )

                await self._fc.goto_position(
                    wp.latitude, wp.longitude, wp.altitude_m, wp.speed_ms
                )

                # Wait until arrival
                await self._wait_for_arrival(wp)

                # Loiter if requested
                if wp.loiter_time_s > 0:
                    logger.info("[%s] Loitering at '%s' for %.1fs.",
                                self._drone_id, wp.name, wp.loiter_time_s)
                    await asyncio.sleep(wp.loiter_time_s)

                mission.current_index += 1

            if mission.repeat:
                mission.current_index = 0
                logger.info("[%s] Mission '%s' repeating.", self._drone_id, mission.name)
                await self._execute_mission(mission)
            else:
                logger.info("[%s] Mission '%s' complete.", self._drone_id, mission.name)
                self._active_mission = None

        except asyncio.CancelledError:
            logger.info("[%s] Mission '%s' cancelled.", self._drone_id, mission.name)
            raise

    async def _wait_for_arrival(self, waypoint: Waypoint, timeout_s: float = 120.0) -> None:
        """
        Block until the drone arrives within waypoint_radius_m of the target,
        or until the timeout expires.

        Args:
            waypoint:  The target waypoint.
            timeout_s: Maximum seconds to wait before giving up.
        """
        deadline = asyncio.get_event_loop().time() + timeout_s
        while asyncio.get_event_loop().time() < deadline:
            state = await self._bridge.vehicle_state.snapshot()
            dist = self._haversine_distance(
                state["latitude"], state["longitude"],
                waypoint.latitude, waypoint.longitude,
            )
            if dist <= self._waypoint_radius_m:
                logger.debug("[%s] Arrived at '%s' (dist=%.1fm).",
                             self._drone_id, waypoint.name, dist)
                return
            await asyncio.sleep(0.5)
        logger.warning("[%s] Timeout waiting for arrival at '%s'.",
                       self._drone_id, waypoint.name)

    @staticmethod
    def _haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """
        Calculate the great-circle distance between two GPS coordinates.

        Args:
            lat1, lon1: Origin coordinates in decimal degrees.
            lat2, lon2: Destination coordinates in decimal degrees.

        Returns:
            Distance in metres.
        """
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)
        a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
        return 2 * EARTH_RADIUS_M * math.asin(math.sqrt(a))
