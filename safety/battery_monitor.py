"""
================================================================================
Project:    River Horizon — Autonomous Drone Fleet Management System
File:       safety/battery_monitor.py
Purpose:    Battery health monitor. Polls battery percentage and voltage from
            the MAVLink bridge and triggers RTH on low battery or immediate
            land on critical battery. Emits warnings to the alert manager.
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

from core.constants import (
    BATTERY_CHECK_INTERVAL_S,
    FaultReason,
    SystemStatus,
)

logger = logging.getLogger(__name__)


class BatteryMonitor:
    """
    Monitors battery state and enforces low/critical battery failsafes.

    Thresholds:
    - low_pct:      Triggers RTH (return to home).
    - critical_pct: Triggers immediate LAND regardless of position.

    Both thresholds are loaded from the drone profile and can be updated
    at runtime via update_thresholds().
    """

    def __init__(
        self,
        drone_id: str,
        low_pct: float,
        critical_pct: float,
        fault_manager,  # FaultManager — forward reference
    ) -> None:
        """
        Initialise the battery monitor.

        Args:
            drone_id:       Drone identifier for logging.
            low_pct:        Battery percentage that triggers RTH.
            critical_pct:   Battery percentage that triggers immediate land.
            fault_manager:  Central fault manager for reporting faults.
        """
        self._drone_id = drone_id
        self._low_pct = low_pct
        self._critical_pct = critical_pct
        self._fault_manager = fault_manager

        self._bridge = None
        self._running: bool = False
        self._status: SystemStatus = SystemStatus.INITIALIZING

        self._last_pct: float = 100.0
        self._last_voltage: float = 0.0
        self._low_triggered: bool = False
        self._critical_triggered: bool = False
        self._last_check_time: Optional[float] = None

    async def initialise(self) -> None:
        """Prepare the battery monitor. Bridge must be set before run()."""
        self._status = SystemStatus.OK
        logger.info(
            "[%s] BatteryMonitor initialised: low=%.0f%%, critical=%.0f%%.",
            self._drone_id, self._low_pct, self._critical_pct,
        )

    def set_bridge(self, bridge) -> None:
        """
        Inject the MAVLink bridge.

        Args:
            bridge: Active MAVLinkBridge instance.
        """
        self._bridge = bridge

    def update_thresholds(self, low_pct: float, critical_pct: float) -> None:
        """
        Update battery thresholds at runtime (e.g. from River Song dashboard).

        Args:
            low_pct:      New low battery threshold percentage.
            critical_pct: New critical battery threshold percentage.
        """
        self._low_pct = low_pct
        self._critical_pct = critical_pct
        logger.info(
            "[%s] Battery thresholds updated: low=%.0f%%, critical=%.0f%%.",
            self._drone_id, low_pct, critical_pct,
        )

    # ------------------------------------------------------------------
    # Background monitoring loop
    # ------------------------------------------------------------------

    async def run(self) -> None:
        """
        Continuously poll battery state and enforce failsafe thresholds.
        Runs until cancelled.
        """
        self._running = True
        logger.info("[%s] Battery monitor loop started.", self._drone_id)

        while self._running:
            try:
                await self._check_battery()
            except asyncio.CancelledError:
                break
            except Exception as exc:  # pylint: disable=broad-except
                logger.error("[%s] Battery check error: %s", self._drone_id, exc)

            await asyncio.sleep(BATTERY_CHECK_INTERVAL_S)

        logger.info("[%s] Battery monitor loop stopped.", self._drone_id)

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def status(self) -> SystemStatus:
        """Current health status of the battery monitor."""
        return self._status

    @property
    def battery_pct(self) -> float:
        """Last known battery percentage."""
        return self._last_pct

    @property
    def battery_voltage(self) -> float:
        """Last known battery voltage in volts."""
        return self._last_voltage

    @property
    def is_low(self) -> bool:
        """True if battery is at or below the low threshold."""
        return self._last_pct <= self._low_pct

    @property
    def is_critical(self) -> bool:
        """True if battery is at or below the critical threshold."""
        return self._last_pct <= self._critical_pct

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    async def _check_battery(self) -> None:
        """
        Read battery state from the bridge and trigger failsafes if needed.
        """
        if self._bridge is None:
            return

        state = await self._bridge.vehicle_state.snapshot()
        pct = state["battery_pct"]
        voltage = state["battery_voltage"]

        self._last_pct = pct
        self._last_voltage = voltage
        self._last_check_time = time.monotonic()

        # Critical threshold — immediate land
        if pct <= self._critical_pct and not self._critical_triggered:
            self._critical_triggered = True
            self._status = SystemStatus.CRITICAL
            logger.critical(
                "[%s] CRITICAL BATTERY: %.0f%% (%.2fV) — triggering immediate LAND.",
                self._drone_id, pct, voltage,
            )
            await self._fault_manager.report_fault(
                FaultReason.CRITICAL_BATTERY,
                f"Critical battery: {pct:.0f}% ({voltage:.2f}V)",
            )
            return

        # Low threshold — RTH
        if pct <= self._low_pct and not self._low_triggered:
            self._low_triggered = True
            self._status = SystemStatus.WARNING
            logger.warning(
                "[%s] LOW BATTERY: %.0f%% (%.2fV) — triggering RTH.",
                self._drone_id, pct, voltage,
            )
            await self._fault_manager.report_fault(
                FaultReason.LOW_BATTERY,
                f"Low battery: {pct:.0f}% ({voltage:.2f}V)",
            )
            return

        # Recovery — reset triggers if battery reading improves (e.g. after swap)
        if pct > self._low_pct + 5.0:
            if self._low_triggered or self._critical_triggered:
                logger.info("[%s] Battery recovered to %.0f%% — resetting failsafe flags.", self._drone_id, pct)
            self._low_triggered = False
            self._critical_triggered = False
            self._status = SystemStatus.OK
