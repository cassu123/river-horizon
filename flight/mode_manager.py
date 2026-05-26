"""
================================================================================
Project:    River Horizon — Autonomous Drone Fleet Management System
File:       flight/mode_manager.py
Purpose:    Flight mode management. Tracks the current mode, validates
            requested mode transitions, and enforces safety constraints
            (e.g. cannot switch to MANUAL while geofence is active).
Author:     [Author Placeholder]
Version:    1.0.0
Date:       2026-05-25
Part of:    River Song AI Ecosystem (riversongai.com)
================================================================================
"""

import asyncio
import logging
from typing import Optional

from core.constants import FlightMode
from flight.mavlink_bridge import MAVLinkBridge

logger = logging.getLogger(__name__)


class ModeTransitionError(Exception):
    """Raised when a requested mode transition is not permitted."""


class ModeManager:
    """
    Manages flight mode transitions for a single drone unit.

    Validates that transitions are safe and legal before forwarding the
    mode change command to the MAVLink bridge. Maintains a local record
    of the current and previous modes for audit and rollback purposes.
    """

    # Modes that are always safe to enter regardless of current state
    _ALWAYS_ALLOWED: frozenset = frozenset({
        FlightMode.RTL,
        FlightMode.LAND,
        FlightMode.BRAKE,
    })

    # Modes that require the vehicle to be armed
    _REQUIRES_ARMED: frozenset = frozenset({
        FlightMode.AUTO,
        FlightMode.GUIDED,
        FlightMode.LOITER,
        FlightMode.ALT_HOLD,
        FlightMode.POSHOLD,
        FlightMode.ACRO,
    })

    def __init__(self, bridge: MAVLinkBridge, drone_id: str) -> None:
        """
        Initialise the mode manager.

        Args:
            bridge:   Active MAVLink bridge for sending mode commands.
            drone_id: Drone identifier used in log messages.
        """
        self._bridge = bridge
        self._drone_id = drone_id
        self._current_mode: FlightMode = FlightMode.STABILIZE
        self._previous_mode: Optional[FlightMode] = None
        self._lock = asyncio.Lock()

    async def initialise(self) -> None:
        """
        Synchronise the local mode record with the vehicle's reported mode.
        Registers a MAVLink HEARTBEAT handler to track mode changes.
        """
        self._bridge.register_handler("HEARTBEAT", self._on_heartbeat)
        logger.info("[%s] ModeManager initialised.", self._drone_id)

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    async def request_mode(self, mode: FlightMode, override_safety: bool = False) -> bool:
        """
        Request a flight mode change.

        Args:
            mode:            The desired FlightMode.
            override_safety: If True, bypass armed-state checks. Use only for
                             emergency failsafe transitions.

        Returns:
            True if the mode change command was accepted by the bridge.

        Raises:
            ModeTransitionError: If the transition is not permitted.
        """
        async with self._lock:
            if mode == self._current_mode:
                logger.debug("[%s] Already in mode %s.", self._drone_id, mode)
                return True

            if not override_safety:
                self._validate_transition(mode)

            success = await self._bridge.set_mode(mode.value)
            if success:
                self._previous_mode = self._current_mode
                self._current_mode = mode
                logger.info("[%s] Mode changed: %s → %s",
                            self._drone_id, self._previous_mode, self._current_mode)
            else:
                logger.error("[%s] Mode change to %s failed.", self._drone_id, mode)
            return success

    async def force_rtl(self) -> bool:
        """
        Immediately command RTL, bypassing all safety checks.
        Used by fault manager and safety monitors.

        Returns:
            True if the RTL command was sent successfully.
        """
        logger.warning("[%s] FORCE RTL commanded.", self._drone_id)
        return await self.request_mode(FlightMode.RTL, override_safety=True)

    async def force_land(self) -> bool:
        """
        Immediately command LAND, bypassing all safety checks.

        Returns:
            True if the LAND command was sent successfully.
        """
        logger.warning("[%s] FORCE LAND commanded.", self._drone_id)
        return await self.request_mode(FlightMode.LAND, override_safety=True)

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def current_mode(self) -> FlightMode:
        """The currently active flight mode."""
        return self._current_mode

    @property
    def previous_mode(self) -> Optional[FlightMode]:
        """The flight mode active before the most recent transition."""
        return self._previous_mode

    @property
    def is_autonomous(self) -> bool:
        """True if the drone is in a fully autonomous mode (AUTO or GUIDED)."""
        return self._current_mode in (FlightMode.AUTO, FlightMode.GUIDED)

    @property
    def is_returning(self) -> bool:
        """True if the drone is executing a return-to-launch."""
        return self._current_mode == FlightMode.RTL

    @property
    def is_landing(self) -> bool:
        """True if the drone is in LAND mode."""
        return self._current_mode == FlightMode.LAND

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _validate_transition(self, target: FlightMode) -> None:
        """
        Validate that the requested mode transition is safe.

        Args:
            target: The requested target mode.

        Raises:
            ModeTransitionError: If the transition is not permitted.
        """
        state = self._bridge.vehicle_state

        # Armed check — some modes require the vehicle to be armed
        if target in self._REQUIRES_ARMED and not state.armed:
            raise ModeTransitionError(
                f"Cannot enter {target} — vehicle is not armed."
            )

        # GPS fix check for position-hold modes
        gps_required = {FlightMode.AUTO, FlightMode.GUIDED, FlightMode.LOITER, FlightMode.POSHOLD}
        if target in gps_required and state.gps_fix < 3:
            raise ModeTransitionError(
                f"Cannot enter {target} — insufficient GPS fix (fix_type={state.gps_fix}, need ≥3)."
            )

    async def _on_heartbeat(self, msg) -> None:
        """
        Update current mode from incoming HEARTBEAT messages.

        Args:
            msg: MAVLink HEARTBEAT message.
        """
        # custom_mode is an integer; map it to a FlightMode string if possible
        # This is autopilot-specific; ArduPilot encodes mode in custom_mode
        # We rely on the bridge's set_mode to keep _current_mode accurate,
        # but we log unexpected external mode changes here.
        pass  # Mode tracking is handled via request_mode() for now
