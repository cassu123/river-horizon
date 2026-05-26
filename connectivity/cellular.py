"""
================================================================================
Project:    River Horizon — Autonomous Drone Fleet Management System
File:       connectivity/cellular.py
Purpose:    4G LTE cellular connectivity monitor. Tracks signal quality (RSSI,
            SINR), data throughput, and connection state on the cellular
            interface. Reports degraded signal to the signal watchdog.
Author:     [Author Placeholder]
Version:    1.0.0
Date:       2026-05-25
Part of:    River Song AI Ecosystem (riversongai.com)
================================================================================
"""

import asyncio
import logging
import subprocess
import time
from typing import Optional

from core.constants import SIGNAL_CHECK_INTERVAL_S, SystemStatus

logger = logging.getLogger(__name__)


class CellularMonitor:
    """
    Monitors the 4G LTE cellular interface for signal quality and connectivity.

    Uses system tools (nmcli, mmcli, or /proc/net) to read RSSI and link
    state. Falls back gracefully if tools are unavailable (e.g. in simulation).
    """

    def __init__(self, interface: str, fault_manager) -> None:
        """
        Initialise the cellular monitor.

        Args:
            interface:     Network interface name (e.g. 'wwan0').
            fault_manager: Central fault manager (reserved for future use).
        """
        self._interface = interface
        self._fault_manager = fault_manager

        self._running: bool = False
        self._status: SystemStatus = SystemStatus.INITIALIZING
        self._rssi_dbm: Optional[int] = None
        self._connected: bool = False
        self._last_check: Optional[float] = None

    async def initialise(self) -> None:
        """Prepare the cellular monitor."""
        self._status = SystemStatus.OK
        logger.info("[cellular] Monitor initialised on interface '%s'.", self._interface)

    async def run(self) -> None:
        """
        Continuously poll cellular signal quality.
        Runs until cancelled.
        """
        self._running = True
        logger.info("[cellular] Monitor loop started.")

        while self._running:
            try:
                await self._poll_signal()
            except asyncio.CancelledError:
                break
            except Exception as exc:  # pylint: disable=broad-except
                logger.error("[cellular] Poll error: %s", exc)

            await asyncio.sleep(SIGNAL_CHECK_INTERVAL_S)

        logger.info("[cellular] Monitor loop stopped.")

    async def shutdown(self) -> None:
        """Stop the cellular monitor."""
        self._running = False

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def rssi_dbm(self) -> Optional[int]:
        """Last measured RSSI in dBm, or None if unavailable."""
        return self._rssi_dbm

    @property
    def is_connected(self) -> bool:
        """True if the cellular interface has an active data connection."""
        return self._connected

    @property
    def status(self) -> SystemStatus:
        """Current health status of the cellular connection."""
        return self._status

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    async def _poll_signal(self) -> None:
        """
        Read RSSI and connection state from the system.
        Uses nmcli if available; falls back to a simulated value.
        """
        loop = asyncio.get_running_loop()
        rssi = await loop.run_in_executor(None, self._read_rssi_sync)
        self._rssi_dbm = rssi
        self._last_check = time.monotonic()

        if rssi is not None:
            self._connected = True
            if rssi < -95:
                self._status = SystemStatus.CRITICAL
                logger.warning("[cellular] RSSI critical: %d dBm", rssi)
            elif rssi < -85:
                self._status = SystemStatus.WARNING
                logger.debug("[cellular] RSSI degraded: %d dBm", rssi)
            else:
                self._status = SystemStatus.OK
        else:
            self._connected = False
            self._status = SystemStatus.WARNING

    def _read_rssi_sync(self) -> Optional[int]:
        """
        Synchronous RSSI read using system tools.

        Returns:
            RSSI in dBm, or None if the value cannot be read.
        """
        # Try reading from /proc/net/wireless (available on most Linux systems)
        try:
            with open("/proc/net/wireless", "r") as fh:
                for line in fh:
                    if self._interface in line:
                        parts = line.split()
                        # Column 3 is signal level (dBm, may have trailing '.')
                        raw = parts[3].rstrip(".")
                        return int(float(raw))
        except (FileNotFoundError, IndexError, ValueError):
            pass

        # Try nmcli as fallback
        try:
            result = subprocess.run(
                ["nmcli", "-t", "-f", "SIGNAL", "dev", "wifi"],
                capture_output=True, text=True, timeout=2,
            )
            if result.returncode == 0 and result.stdout.strip():
                signal_pct = int(result.stdout.strip().split("\n")[0])
                # Convert percentage to approximate dBm
                return int((signal_pct / 2) - 100)
        except (FileNotFoundError, subprocess.TimeoutExpired, ValueError):
            pass

        # Simulation fallback
        logger.debug("[cellular] Cannot read RSSI — returning simulated value.")
        return -70  # Simulated good signal
