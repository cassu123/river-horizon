"""
================================================================================
Project:    River Horizon — Autonomous Drone Fleet Management System
File:       core/fleet_manager.py
Purpose:    Multi-drone fleet manager. Maintains a registry of active drone
            units, aggregates fleet-wide telemetry and fault state, and
            provides a unified interface for the River Song dashboard to
            query and command the entire fleet.
Author:     [Author Placeholder]
Version:    1.0.0
Date:       2026-05-25
Part of:    River Song AI Ecosystem (riversongai.com)
================================================================================
"""

import asyncio
import logging
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

from core.constants import SystemStatus

logger = logging.getLogger(__name__)


@dataclass
class DroneUnit:
    """
    Represents a single registered drone unit in the fleet.

    Each unit has its own MAVLink bridge, flight controller, and safety
    systems. The fleet manager holds references to these for cross-unit
    coordination.
    """
    drone_id: str
    model: str
    registered_at: float = field(default_factory=time.time)
    last_seen: float = field(default_factory=time.time)
    status: SystemStatus = SystemStatus.INITIALIZING

    # Runtime subsystem references (set after unit initialisation)
    flight_controller: Optional[Any] = None
    waypoint_manager: Optional[Any] = None
    mode_manager: Optional[Any] = None
    telemetry_collector: Optional[Any] = None
    fault_manager: Optional[Any] = None
    stream_manager: Optional[Any] = None

    @property
    def is_online(self) -> bool:
        """True if the unit has been seen within the last 10 seconds."""
        return time.time() - self.last_seen < 10.0

    def touch(self) -> None:
        """Update the last-seen timestamp."""
        self.last_seen = time.time()


class FleetManager:
    """
    Multi-drone fleet registry and coordinator.

    Maintains a registry of all active drone units. Provides:
    - Unit registration and deregistration
    - Fleet-wide status aggregation
    - Cross-unit command broadcasting (e.g. "land all drones")
    - Fleet telemetry summary for the River Song dashboard

    In the current architecture, each drone runs its own River Horizon
    process on its companion computer. The fleet manager on the River Song
    server aggregates status from all units via the API.

    For single-drone deployments, the fleet manager manages one unit and
    adds negligible overhead.
    """

    def __init__(self) -> None:
        """Initialise an empty fleet registry."""
        self._units: Dict[str, DroneUnit] = {}
        self._lock = asyncio.Lock()

    # ------------------------------------------------------------------
    # Unit registration
    # ------------------------------------------------------------------

    async def register_unit(self, unit: DroneUnit) -> None:
        """
        Register a drone unit with the fleet.

        Args:
            unit: The DroneUnit to register.
        """
        async with self._lock:
            self._units[unit.drone_id] = unit
        logger.info(
            "Fleet: registered unit '%s' (%s). Fleet size: %d.",
            unit.drone_id, unit.model, len(self._units),
        )

    async def deregister_unit(self, drone_id: str) -> None:
        """
        Remove a drone unit from the fleet registry.

        Args:
            drone_id: The ID of the unit to remove.
        """
        async with self._lock:
            removed = self._units.pop(drone_id, None)
        if removed:
            logger.info("Fleet: deregistered unit '%s'.", drone_id)
        else:
            logger.warning("Fleet: deregister called for unknown unit '%s'.", drone_id)

    def get_unit(self, drone_id: str) -> Optional[DroneUnit]:
        """
        Retrieve a registered drone unit by ID.

        Args:
            drone_id: The drone unit identifier.

        Returns:
            The DroneUnit if registered, otherwise None.
        """
        return self._units.get(drone_id)

    def list_units(self) -> List[DroneUnit]:
        """
        Return all registered drone units.

        Returns:
            List of DroneUnit objects, sorted by drone_id.
        """
        return sorted(self._units.values(), key=lambda u: u.drone_id)

    # ------------------------------------------------------------------
    # Fleet-wide commands
    # ------------------------------------------------------------------

    async def land_all(self) -> Dict[str, bool]:
        """
        Command all active drone units to land immediately.

        Returns:
            Dictionary mapping drone_id → success bool.
        """
        logger.warning("Fleet: LAND ALL commanded.")
        results = {}
        tasks = []
        for unit in self._units.values():
            if unit.flight_controller is not None:
                tasks.append(self._land_unit(unit))
        outcomes = await asyncio.gather(*tasks, return_exceptions=True)
        for unit, outcome in zip(self._units.values(), outcomes):
            results[unit.drone_id] = not isinstance(outcome, Exception)
        return results

    async def rth_all(self) -> Dict[str, bool]:
        """
        Command all active drone units to return to home.

        Returns:
            Dictionary mapping drone_id → success bool.
        """
        logger.warning("Fleet: RTH ALL commanded.")
        results = {}
        tasks = []
        for unit in self._units.values():
            if unit.mode_manager is not None:
                tasks.append(unit.mode_manager.force_rtl())
        outcomes = await asyncio.gather(*tasks, return_exceptions=True)
        for unit, outcome in zip(self._units.values(), outcomes):
            results[unit.drone_id] = outcome is True
        return results

    async def abort_all_missions(self) -> Dict[str, bool]:
        """
        Abort active missions on all drone units.

        Returns:
            Dictionary mapping drone_id → success bool.
        """
        logger.info("Fleet: aborting all missions.")
        results = {}
        for unit in self._units.values():
            if unit.waypoint_manager is not None:
                try:
                    await unit.waypoint_manager.abort_mission()
                    results[unit.drone_id] = True
                except Exception as exc:  # pylint: disable=broad-except
                    logger.error("Fleet: abort mission failed for '%s': %s", unit.drone_id, exc)
                    results[unit.drone_id] = False
        return results

    # ------------------------------------------------------------------
    # Fleet status
    # ------------------------------------------------------------------

    async def fleet_status(self) -> Dict[str, Any]:
        """
        Build a fleet-wide status summary for the River Song dashboard.

        Returns:
            Dictionary with fleet summary and per-unit status.
        """
        units_status = []
        for unit in self.list_units():
            unit_data: Dict[str, Any] = {
                "drone_id": unit.drone_id,
                "model": unit.model,
                "online": unit.is_online,
                "status": unit.status,
                "registered_at": unit.registered_at,
                "last_seen": unit.last_seen,
            }

            # Add live telemetry if available
            if unit.telemetry_collector is not None:
                packet = unit.telemetry_collector.last_packet
                if packet:
                    unit_data["telemetry"] = packet

            # Add fault state if available
            if unit.fault_manager is not None:
                unit_data["active_faults"] = [
                    r.value for r in unit.fault_manager.active_faults.keys()
                ]

            units_status.append(unit_data)

        online_count = sum(1 for u in self._units.values() if u.is_online)
        return {
            "fleet_size": len(self._units),
            "online": online_count,
            "offline": len(self._units) - online_count,
            "units": units_status,
        }

    @property
    def unit_count(self) -> int:
        """Total number of registered drone units."""
        return len(self._units)

    @property
    def online_count(self) -> int:
        """Number of units that have been seen recently."""
        return sum(1 for u in self._units.values() if u.is_online)

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    @staticmethod
    async def _land_unit(unit: DroneUnit) -> bool:
        """
        Issue a land command to a single unit.

        Args:
            unit: The DroneUnit to land.

        Returns:
            True if the land command was accepted.
        """
        try:
            return await unit.flight_controller.land()
        except Exception as exc:  # pylint: disable=broad-except
            logger.error("Fleet: land failed for '%s': %s", unit.drone_id, exc)
            return False


# ---------------------------------------------------------------------------
# Module-level singleton
# ---------------------------------------------------------------------------
fleet = FleetManager()
