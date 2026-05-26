"""
================================================================================
Project:    River Horizon — Autonomous Drone Fleet Management System
File:       flight/return_home.py
Purpose:    Return-to-Home (RTH) controller. Executes RTH sequences triggered
            by low battery, signal loss, geofence breach, or manual command.
            Climbs to safe RTH altitude before returning to prevent obstacles.
Author:     [Author Placeholder]
Version:    1.0.0
Date:       2026-05-25
Part of:    River Song AI Ecosystem (riversongai.com)
================================================================================
"""

import asyncio
import logging
import time
from typing import Optional

from core.constants import FlightMode, FaultReason
from flight.mavlink_bridge import MAVLinkBridge
from flight.mode_manager import ModeManager

logger = logging.getLogger(__name__)


class ReturnHomeController:
    """
    Manages Return-to-Home sequences for a single drone unit.

    RTH is triggered automatically by the fault manager when:
    - Battery drops below the low threshold
    - Signal is lost for longer than the configured timeout
    - A geofence breach is detected

    The controller climbs to the configured RTH altitude before engaging
    RTL mode to avoid obstacles during the return leg.
    """

    def __init__(
        self,
        bridge: MAVLinkBridge,
        mode_manager: ModeManager,
        rth_altitude_m: float,
        fault_manager,  # FaultManager — forward reference
    ) -> None:
        """
        Initialise the RTH controller.

        Args:
            bridge:          Active MAVLink bridge.
            mode_manager:    Mode manager for issuing RTL/LAND commands.
            rth_altitude_m:  Minimum altitude to climb to before returning.
            fault_manager:   Central fault manager for status reporting.
        """
        self._bridge = bridge
        self._mode_manager = mode_manager
        self._rth_altitude_m = rth_altitude_m
        self._fault_manager = fault_manager
        self._drone_id = bridge._drone_id

        self._rth_active: bool = False
        self._rth_reason: Optional[FaultReason] = None
        self._rth_start_time: Optional[float] = None
        self._lock = asyncio.Lock()

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    async def execute_rth(self, reason: FaultReason = FaultReason.MANUAL_OVERRIDE) -> bool:
        """
        Execute a Return-to-Home sequence.

        If the drone is below the RTH altitude, it will climb first before
        engaging RTL mode. This prevents low-altitude returns through obstacles.

        Args:
            reason: The fault reason that triggered this RTH.

        Returns:
            True if the RTH sequence was initiated successfully.
        """
        async with self._lock:
            if self._rth_active:
                logger.warning(
                    "[%s] RTH already active (reason=%s). Ignoring duplicate trigger.",
                    self._drone_id, self._rth_reason,
                )
                return True

            self._rth_active = True
            self._rth_reason = reason
            self._rth_start_time = time.monotonic()

        logger.warning(
            "[%s] *** RTH INITIATED *** reason=%s, rth_altitude=%.1fm",
            self._drone_id, reason.value, self._rth_altitude_m,
        )

        state = await self._bridge.vehicle_state.snapshot()

        # If not armed / not airborne, nothing to do
        if not state["armed"]:
            logger.info("[%s] RTH: vehicle not armed — no action needed.", self._drone_id)
            await self._clear_rth()
            return True

        # Climb to RTH altitude if below it
        if state["relative_altitude_m"] < self._rth_altitude_m - 2.0:
            logger.info(
                "[%s] RTH: climbing from %.1fm to RTH altitude %.1fm.",
                self._drone_id, state["relative_altitude_m"], self._rth_altitude_m,
            )
            await self._climb_to_rth_altitude()

        # Engage RTL mode
        success = await self._mode_manager.force_rtl()
        if success:
            logger.info("[%s] RTL mode engaged. Returning to home.", self._drone_id)
        else:
            logger.error("[%s] Failed to engage RTL — attempting LAND as fallback.", self._drone_id)
            success = await self._mode_manager.force_land()

        return success

    async def cancel_rth(self) -> bool:
        """
        Cancel an active RTH sequence (operator override).

        Returns:
            True if RTH was active and has been cancelled.
        """
        async with self._lock:
            if not self._rth_active:
                logger.debug("[%s] cancel_rth called but RTH is not active.", self._drone_id)
                return False
            await self._clear_rth()
            logger.info("[%s] RTH cancelled by operator.", self._drone_id)
            return True

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def is_rth_active(self) -> bool:
        """True if an RTH sequence is currently in progress."""
        return self._rth_active

    @property
    def rth_reason(self) -> Optional[FaultReason]:
        """The fault reason that triggered the current RTH, or None."""
        return self._rth_reason

    @property
    def rth_duration_s(self) -> Optional[float]:
        """Elapsed seconds since RTH was initiated, or None if not active."""
        if self._rth_start_time is None:
            return None
        return time.monotonic() - self._rth_start_time

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    async def _climb_to_rth_altitude(self, timeout_s: float = 30.0) -> None:
        """
        Command the vehicle to climb to the RTH altitude and wait for arrival.

        Args:
            timeout_s: Maximum seconds to wait for the climb to complete.
        """
        try:
            from pymavlink import mavutil
            await self._bridge.send_command_long(
                mavutil.mavlink.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT,
                param1=1,                       # climb
                param7=self._rth_altitude_m,
            )
        except ImportError:
            logger.debug("[%s] Simulation: climb to %.1fm", self._drone_id, self._rth_altitude_m)

        # Wait until altitude is reached or timeout
        deadline = asyncio.get_event_loop().time() + timeout_s
        while asyncio.get_event_loop().time() < deadline:
            state = await self._bridge.vehicle_state.snapshot()
            if state["relative_altitude_m"] >= self._rth_altitude_m - 2.0:
                logger.info("[%s] RTH altitude reached: %.1fm.",
                            self._drone_id, state["relative_altitude_m"])
                return
            await asyncio.sleep(0.5)

        logger.warning(
            "[%s] Timeout waiting for RTH altitude climb — proceeding with RTL anyway.",
            self._drone_id,
        )

    async def _clear_rth(self) -> None:
        """Reset RTH state flags."""
        self._rth_active = False
        self._rth_reason = None
        self._rth_start_time = None
