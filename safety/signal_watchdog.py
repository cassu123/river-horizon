"""
================================================================================
Project:    River Horizon — Autonomous Drone Fleet Management System
File:       safety/signal_watchdog.py
Purpose:    Signal loss watchdog. Monitors MAVLink heartbeat timestamps and
            cellular RSSI. Triggers RTH if the control link is lost for longer
            than the configured timeout. Monitors both the MAVLink link and
            the 4G LTE cellular connection independently.
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
    SIGNAL_CHECK_INTERVAL_S,
    MAVLINK_HEARTBEAT_TIMEOUT_S,
    FaultReason,
    SystemStatus,
)

logger = logging.getLogger(__name__)


class SignalWatchdog:
    """
    Dual-link signal loss watchdog.

    Monitors:
    1. MAVLink heartbeat — detects loss of GCS ↔ FC link.
    2. Cellular RSSI — detects degraded or lost 4G LTE connection.

    On signal loss exceeding the configured timeout, a SIGNAL_LOSS fault
    is reported to the fault manager, which triggers RTH.
    """

    def __init__(
        self,
        drone_id: str,
        timeout_s: float,
        min_rssi_dbm: int,
        fault_manager,  # FaultManager — forward reference
    ) -> None:
        """
        Initialise the signal watchdog.

        Args:
            drone_id:       Drone identifier for logging.
            timeout_s:      Seconds of signal loss before triggering RTH.
            min_rssi_dbm:   Minimum acceptable RSSI in dBm (e.g. -90).
            fault_manager:  Central fault manager for reporting faults.
        """
        self._drone_id = drone_id
        self._timeout_s = timeout_s
        self._min_rssi_dbm = min_rssi_dbm
        self._fault_manager = fault_manager

        self._bridge = None
        self._cellular = None  # CellularMonitor — injected after init

        self._running: bool = False
        self._status: SystemStatus = SystemStatus.INITIALIZING
        self._signal_lost_at: Optional[float] = None
        self._fault_triggered: bool = False

    async def initialise(self) -> None:
        """Prepare the watchdog. Bridge must be set before run()."""
        self._status = SystemStatus.OK
        logger.info(
            "[%s] SignalWatchdog initialised: timeout=%.1fs, min_rssi=%ddBm.",
            self._drone_id, self._timeout_s, self._min_rssi_dbm,
        )

    def set_bridge(self, bridge) -> None:
        """
        Inject the MAVLink bridge.

        Args:
            bridge: Active MAVLinkBridge instance.
        """
        self._bridge = bridge

    def set_cellular(self, cellular) -> None:
        """
        Inject the cellular monitor for RSSI checks.

        Args:
            cellular: Active CellularMonitor instance.
        """
        self._cellular = cellular

    # ------------------------------------------------------------------
    # Background monitoring loop
    # ------------------------------------------------------------------

    async def run(self) -> None:
        """
        Continuously monitor signal quality and enforce the loss timeout.
        Runs until cancelled.
        """
        self._running = True
        logger.info("[%s] Signal watchdog loop started.", self._drone_id)

        while self._running:
            try:
                await self._check_signal()
            except asyncio.CancelledError:
                break
            except Exception as exc:  # pylint: disable=broad-except
                logger.error("[%s] Signal check error: %s", self._drone_id, exc)

            await asyncio.sleep(SIGNAL_CHECK_INTERVAL_S)

        logger.info("[%s] Signal watchdog loop stopped.", self._drone_id)

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def status(self) -> SystemStatus:
        """Current health status of the signal watchdog."""
        return self._status

    @property
    def is_signal_lost(self) -> bool:
        """True if signal loss has been detected and the timeout has elapsed."""
        return self._fault_triggered

    @property
    def signal_lost_duration_s(self) -> Optional[float]:
        """Seconds since signal was first lost, or None if signal is present."""
        if self._signal_lost_at is None:
            return None
        return time.monotonic() - self._signal_lost_at

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    async def _check_signal(self) -> None:
        """
        Check MAVLink heartbeat age and cellular RSSI.
        Trigger RTH if signal has been lost beyond the timeout.
        """
        now = time.monotonic()
        signal_ok = True

        # --- MAVLink heartbeat check ---
        if self._bridge is not None:
            state = await self._bridge.vehicle_state.snapshot()
            last_hb = state["last_heartbeat"]
            if last_hb > 0:
                hb_age = now - last_hb
                if hb_age > MAVLINK_HEARTBEAT_TIMEOUT_S:
                    logger.warning(
                        "[%s] MAVLink heartbeat timeout: last seen %.1fs ago.",
                        self._drone_id, hb_age,
                    )
                    signal_ok = False

        # --- Cellular RSSI check ---
        if self._cellular is not None:
            rssi = self._cellular.rssi_dbm
            if rssi is not None and rssi < self._min_rssi_dbm:
                logger.warning(
                    "[%s] Cellular RSSI degraded: %ddBm (min=%ddBm).",
                    self._drone_id, rssi, self._min_rssi_dbm,
                )
                signal_ok = False

        # --- Timeout logic ---
        if not signal_ok:
            if self._signal_lost_at is None:
                self._signal_lost_at = now
                self._status = SystemStatus.WARNING
                logger.warning("[%s] Signal degraded — starting loss timer.", self._drone_id)

            lost_duration = now - self._signal_lost_at
            if lost_duration >= self._timeout_s and not self._fault_triggered:
                self._fault_triggered = True
                self._status = SystemStatus.CRITICAL
                logger.critical(
                    "[%s] SIGNAL LOST for %.1fs — triggering RTH.",
                    self._drone_id, lost_duration,
                )
                await self._fault_manager.report_fault(
                    FaultReason.SIGNAL_LOSS,
                    f"Signal lost for {lost_duration:.1f}s (timeout={self._timeout_s:.1f}s)",
                )
        else:
            # Signal restored
            if self._signal_lost_at is not None:
                logger.info("[%s] Signal restored.", self._drone_id)
            self._signal_lost_at = None
            self._fault_triggered = False
            self._status = SystemStatus.OK
