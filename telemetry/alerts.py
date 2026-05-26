"""
================================================================================
Project:    River Horizon — Autonomous Drone Fleet Management System
File:       telemetry/alerts.py
Purpose:    Alert manager. Evaluates incoming telemetry packets against
            configurable thresholds and sends alert notifications upstream
            to the River Song API. Supports rate-limiting to prevent alert
            storms during sustained fault conditions.
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

from core.constants import FaultReason, TelemetryField

logger = logging.getLogger(__name__)

# Minimum seconds between repeated alerts for the same condition
_ALERT_COOLDOWN_S = 30.0


@dataclass
class Alert:
    """A single alert event."""
    drone_id: str
    level: str          # "WARNING" | "CRITICAL"
    reason: str
    message: str
    timestamp: float = field(default_factory=time.time)


class AlertManager:
    """
    Threshold-based alert manager.

    Evaluates each telemetry packet against a set of alert rules and
    sends notifications to the River Song API when thresholds are crossed.
    Rate-limiting prevents duplicate alerts during sustained conditions.
    """

    def __init__(self, drone_id: str, api_client) -> None:
        """
        Initialise the alert manager.

        Args:
            drone_id:   Drone identifier included in all alerts.
            api_client: River Song API client for sending alert notifications.
        """
        self._drone_id = drone_id
        self._api_client = api_client
        self._alert_history: List[Alert] = []
        self._last_alert_time: Dict[str, float] = {}

    # ------------------------------------------------------------------
    # Telemetry evaluation
    # ------------------------------------------------------------------

    async def check_telemetry(self, packet: Dict[str, Any]) -> None:
        """
        Evaluate a telemetry packet against all alert rules.

        Args:
            packet: Telemetry packet dictionary from TelemetryCollector.
        """
        battery_pct = packet.get(TelemetryField.BATTERY_PCT, 100.0)
        rssi = packet.get(TelemetryField.SIGNAL_RSSI)
        gps_fix = packet.get(TelemetryField.GPS_FIX, 3)
        gps_sats = packet.get(TelemetryField.GPS_SATS, 10)

        # Battery warnings
        if battery_pct is not None:
            if battery_pct <= 10.0:
                await self._maybe_send_alert(
                    "battery_critical", "CRITICAL",
                    f"Battery critical: {battery_pct:.0f}%",
                )
            elif battery_pct <= 20.0:
                await self._maybe_send_alert(
                    "battery_low", "WARNING",
                    f"Battery low: {battery_pct:.0f}%",
                )

        # GPS degradation
        if gps_fix < 3:
            await self._maybe_send_alert(
                "gps_fix_low", "WARNING",
                f"GPS fix degraded: fix_type={gps_fix}, sats={gps_sats}",
            )

        # RSSI degradation
        if rssi is not None and rssi < -85:
            await self._maybe_send_alert(
                "rssi_low", "WARNING",
                f"Cellular RSSI low: {rssi} dBm",
            )

    # ------------------------------------------------------------------
    # Fault alerts (called directly by FaultManager)
    # ------------------------------------------------------------------

    async def send_fault_alert(self, reason: FaultReason, message: str) -> None:
        """
        Send an alert for a fault event reported by the fault manager.

        Args:
            reason:  The FaultReason that triggered the fault.
            message: Human-readable fault description.
        """
        alert = Alert(
            drone_id=self._drone_id,
            level="CRITICAL",
            reason=reason.value,
            message=message,
        )
        self._alert_history.append(alert)
        logger.warning("[%s] FAULT ALERT: %s — %s", self._drone_id, reason.value, message)

        try:
            await self._api_client.send_alert(alert.__dict__)
        except Exception as exc:  # pylint: disable=broad-except
            logger.error("[%s] Failed to send fault alert upstream: %s", self._drone_id, exc)

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def alert_history(self) -> List[Alert]:
        """Complete list of alerts sent this session."""
        return list(self._alert_history)

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    async def _maybe_send_alert(self, key: str, level: str, message: str) -> None:
        """
        Send an alert if the cooldown period for this key has elapsed.

        Args:
            key:     Unique string identifying this alert type.
            level:   Alert severity ("WARNING" or "CRITICAL").
            message: Human-readable alert message.
        """
        now = time.time()
        last = self._last_alert_time.get(key, 0.0)
        if now - last < _ALERT_COOLDOWN_S:
            return

        self._last_alert_time[key] = now
        alert = Alert(
            drone_id=self._drone_id,
            level=level,
            reason=key,
            message=message,
        )
        self._alert_history.append(alert)
        logger.warning("[%s] ALERT [%s]: %s", self._drone_id, level, message)

        try:
            await self._api_client.send_alert(alert.__dict__)
        except Exception as exc:  # pylint: disable=broad-except
            logger.debug("[%s] Alert upstream send failed: %s", self._drone_id, exc)
