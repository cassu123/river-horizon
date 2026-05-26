"""
================================================================================
Project:    River Horizon — Autonomous Drone Fleet Management System
File:       telemetry/collector.py
Purpose:    Telemetry collector. Reads vehicle state from the MAVLink bridge
            at a configurable rate, packages it into telemetry packets, writes
            to the structured logger, checks alert thresholds, and pushes
            packets upstream to the River Song API.
Author:     [Author Placeholder]
Version:    1.0.0
Date:       2026-05-25
Part of:    River Song AI Ecosystem (riversongai.com)
================================================================================
"""

import asyncio
import logging
import time
from typing import Any, Dict, Optional

from core.constants import TelemetryField

logger = logging.getLogger(__name__)


class TelemetryCollector:
    """
    Periodic telemetry collector for a single drone unit.

    Runs at a configurable rate (default 10 Hz), reads the latest vehicle
    state snapshot from the MAVLink bridge, and distributes it to:
    - TelemetryLogger (structured on-disk log)
    - AlertManager (threshold-based alerting)
    - RiverSongAPIClient (upstream push to River Song dashboard)
    """

    def __init__(
        self,
        bridge,             # MAVLinkBridge
        drone_id: str,
        tel_logger,         # TelemetryLogger
        alert_manager,      # AlertManager
        api_client,         # RiverSongAPIClient
        rate_hz: int = 10,
    ) -> None:
        """
        Initialise the telemetry collector.

        Args:
            bridge:        Active MAVLink bridge for vehicle state reads.
            drone_id:      Drone identifier included in every packet.
            tel_logger:    Telemetry logger for on-disk structured logging.
            alert_manager: Alert manager for threshold-based notifications.
            api_client:    River Song API client for upstream telemetry push.
            rate_hz:       Collection and publish rate in Hz.
        """
        self._bridge = bridge
        self._drone_id = drone_id
        self._tel_logger = tel_logger
        self._alert_manager = alert_manager
        self._api_client = api_client
        self._rate_hz = rate_hz
        self._interval_s = 1.0 / rate_hz

        self._running: bool = False
        self._packet_count: int = 0
        self._last_packet: Optional[Dict[str, Any]] = None

    # ------------------------------------------------------------------
    # Background collection loop
    # ------------------------------------------------------------------

    async def run(self) -> None:
        """
        Collect and distribute telemetry at the configured rate.
        Runs until shutdown() is called or the task is cancelled.
        """
        self._running = True
        logger.info(
            "[%s] Telemetry collector started at %d Hz.", self._drone_id, self._rate_hz
        )

        while self._running:
            start = time.monotonic()
            try:
                packet = await self._collect_packet()
                await self._distribute(packet)
                self._last_packet = packet
                self._packet_count += 1
            except asyncio.CancelledError:
                break
            except Exception as exc:  # pylint: disable=broad-except
                logger.error("[%s] Telemetry collection error: %s", self._drone_id, exc)

            # Maintain rate
            elapsed = time.monotonic() - start
            sleep_time = max(0.0, self._interval_s - elapsed)
            await asyncio.sleep(sleep_time)

        logger.info(
            "[%s] Telemetry collector stopped. Total packets: %d.",
            self._drone_id, self._packet_count,
        )

    async def shutdown(self) -> None:
        """Stop the collection loop cleanly."""
        self._running = False

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def last_packet(self) -> Optional[Dict[str, Any]]:
        """The most recently collected telemetry packet, or None."""
        return self._last_packet

    @property
    def packet_count(self) -> int:
        """Total number of telemetry packets collected this session."""
        return self._packet_count

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    async def _collect_packet(self) -> Dict[str, Any]:
        """
        Build a telemetry packet from the current vehicle state.

        Returns:
            Dictionary containing all telemetry fields for this timestamp.
        """
        state = await self._bridge.vehicle_state.snapshot()
        return {
            TelemetryField.DRONE_ID:        self._drone_id,
            TelemetryField.TIMESTAMP:       time.time(),
            TelemetryField.LATITUDE:        state["latitude"],
            TelemetryField.LONGITUDE:       state["longitude"],
            TelemetryField.ALTITUDE_M:      state["altitude_m"],
            TelemetryField.HEADING_DEG:     state["heading_deg"],
            TelemetryField.SPEED_MS:        state["speed_ms"],
            TelemetryField.BATTERY_PCT:     state["battery_pct"],
            TelemetryField.BATTERY_VOLTAGE: state["battery_voltage"],
            TelemetryField.FLIGHT_MODE:     state["flight_mode"],
            TelemetryField.SIGNAL_RSSI:     None,   # Populated by cellular monitor
            TelemetryField.GPS_FIX:         state["gps_fix"],
            TelemetryField.GPS_SATS:        state["gps_sats"],
            TelemetryField.ARMED:           state["armed"],
            TelemetryField.SYSTEM_STATUS:   state["system_status"],
        }

    async def _distribute(self, packet: Dict[str, Any]) -> None:
        """
        Send the telemetry packet to all registered consumers.

        Args:
            packet: The telemetry packet to distribute.
        """
        # Log to disk
        try:
            await self._tel_logger.log(packet)
        except Exception as exc:  # pylint: disable=broad-except
            logger.warning("[%s] Telemetry log write failed: %s", self._drone_id, exc)

        # Check alert thresholds
        try:
            await self._alert_manager.check_telemetry(packet)
        except Exception as exc:  # pylint: disable=broad-except
            logger.warning("[%s] Alert check failed: %s", self._drone_id, exc)

        # Push to River Song API (best-effort, non-blocking)
        try:
            await self._api_client.push_telemetry(packet)
        except Exception as exc:  # pylint: disable=broad-except
            logger.debug("[%s] Telemetry upstream push failed: %s", self._drone_id, exc)
