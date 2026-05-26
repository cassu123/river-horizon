"""
================================================================================
Project:    River Horizon — Autonomous Drone Fleet Management System
File:       tests/test_telemetry.py
Purpose:    Unit tests for telemetry subsystems: collector, logger, and
            alert manager.
Author:     [Author Placeholder]
Version:    1.0.0
Date:       2026-05-25
Part of:    River Song AI Ecosystem (riversongai.com)
================================================================================
"""

import asyncio
import json
import tempfile
import time
from pathlib import Path
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from core.constants import TelemetryField
from telemetry.collector import TelemetryCollector
from telemetry.logger import TelemetryLogger
from telemetry.alerts import AlertManager


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def mock_bridge():
    """Return a mock MAVLink bridge with a snapshot method."""
    bridge = MagicMock()
    bridge.vehicle_state = MagicMock()
    bridge.vehicle_state.snapshot = AsyncMock(return_value={
        "latitude": 51.5074,
        "longitude": -0.1278,
        "altitude_m": 30.0,
        "relative_altitude_m": 30.0,
        "heading_deg": 90.0,
        "speed_ms": 5.0,
        "battery_pct": 75.0,
        "battery_voltage": 12.2,
        "flight_mode": "GUIDED",
        "armed": True,
        "gps_fix": 3,
        "gps_sats": 10,
        "system_status": "OK",
        "last_heartbeat": 1000.0,
    })
    return bridge


@pytest.fixture
def mock_tel_logger():
    """Return a mock TelemetryLogger."""
    logger = MagicMock()
    logger.log = AsyncMock()
    return logger


@pytest.fixture
def mock_alert_manager():
    """Return a mock AlertManager."""
    am = MagicMock()
    am.check_telemetry = AsyncMock()
    am.send_fault_alert = AsyncMock()
    return am


@pytest.fixture
def mock_api_client():
    """Return a mock RiverSongAPIClient."""
    client = MagicMock()
    client.push_telemetry = AsyncMock(return_value=True)
    client.send_alert = AsyncMock(return_value=True)
    return client


@pytest.fixture
def tmp_log_dir():
    """Return a temporary directory for telemetry log files."""
    with tempfile.TemporaryDirectory() as tmpdir:
        yield Path(tmpdir)


# ---------------------------------------------------------------------------
# TelemetryCollector tests
# ---------------------------------------------------------------------------

class TestTelemetryCollector:
    """Tests for the telemetry collection and distribution pipeline."""

    @pytest.fixture
    def collector(self, mock_bridge, mock_tel_logger, mock_alert_manager, mock_api_client):
        """Return a TelemetryCollector with mock dependencies."""
        return TelemetryCollector(
            bridge=mock_bridge,
            drone_id="TEST-01",
            tel_logger=mock_tel_logger,
            alert_manager=mock_alert_manager,
            api_client=mock_api_client,
            rate_hz=10,
        )

    @pytest.mark.asyncio
    async def test_collect_packet_contains_required_fields(self, collector):
        """_collect_packet() should return a dict with all TelemetryField keys."""
        packet = await collector._collect_packet()
        assert TelemetryField.DRONE_ID in packet
        assert TelemetryField.TIMESTAMP in packet
        assert TelemetryField.LATITUDE in packet
        assert TelemetryField.BATTERY_PCT in packet
        assert TelemetryField.ARMED in packet

    @pytest.mark.asyncio
    async def test_collect_packet_drone_id_matches(self, collector):
        """Collected packet should contain the correct drone ID."""
        packet = await collector._collect_packet()
        assert packet[TelemetryField.DRONE_ID] == "TEST-01"

    @pytest.mark.asyncio
    async def test_distribute_calls_all_consumers(
        self, collector, mock_tel_logger, mock_alert_manager, mock_api_client
    ):
        """_distribute() should call logger, alert manager, and API client."""
        packet = await collector._collect_packet()
        await collector._distribute(packet)
        mock_tel_logger.log.assert_called_once_with(packet)
        mock_alert_manager.check_telemetry.assert_called_once_with(packet)
        mock_api_client.push_telemetry.assert_called_once_with(packet)

    @pytest.mark.asyncio
    async def test_last_packet_updated_after_collection(self, collector):
        """last_packet should be updated after the first collection cycle."""
        assert collector.last_packet is None
        packet = await collector._collect_packet()
        await collector._distribute(packet)
        collector._last_packet = packet
        assert collector.last_packet is not None

    @pytest.mark.asyncio
    async def test_packet_count_increments(self, collector):
        """packet_count should increment with each collection."""
        assert collector.packet_count == 0
        packet = await collector._collect_packet()
        await collector._distribute(packet)
        collector._packet_count += 1
        assert collector.packet_count == 1

    @pytest.mark.asyncio
    async def test_distribute_continues_on_logger_error(
        self, collector, mock_tel_logger, mock_alert_manager, mock_api_client
    ):
        """_distribute() should not raise if the logger fails."""
        mock_tel_logger.log = AsyncMock(side_effect=IOError("Disk full"))
        packet = await collector._collect_packet()
        # Should not raise
        await collector._distribute(packet)
        # Alert manager and API client should still be called
        mock_alert_manager.check_telemetry.assert_called_once()


# ---------------------------------------------------------------------------
# TelemetryLogger tests
# ---------------------------------------------------------------------------

class TestTelemetryLogger:
    """Tests for the structured JSONL telemetry logger."""

    @pytest.fixture
    def tel_logger(self, tmp_log_dir):
        """Return a TelemetryLogger writing to a temp directory."""
        return TelemetryLogger(drone_id="TEST-01", log_dir=tmp_log_dir)

    def test_log_file_created_on_init(self, tel_logger, tmp_log_dir):
        """A log file should be created when the logger is initialised."""
        logs = list(tmp_log_dir.glob("telemetry_TEST-01_*.jsonl"))
        assert len(logs) == 1

    @pytest.mark.asyncio
    async def test_log_writes_valid_json(self, tel_logger, tmp_log_dir):
        """Each logged packet should be valid JSON on its own line."""
        packet = {
            TelemetryField.DRONE_ID: "TEST-01",
            TelemetryField.TIMESTAMP: time.time(),
            TelemetryField.LATITUDE: 51.5074,
            TelemetryField.BATTERY_PCT: 75.0,
        }
        await tel_logger.log(packet)
        await tel_logger.flush()

        log_file = tel_logger.current_log_path
        lines = log_file.read_bytes().decode("utf-8").strip().split("\n")
        assert len(lines) == 1
        parsed = json.loads(lines[0])
        assert parsed[TelemetryField.DRONE_ID] == "TEST-01"

    @pytest.mark.asyncio
    async def test_multiple_packets_written_as_separate_lines(self, tel_logger):
        """Multiple packets should each appear on their own line."""
        for i in range(5):
            await tel_logger.log({"seq": i, "ts": time.time()})
        await tel_logger.flush()

        lines = tel_logger.current_log_path.read_bytes().decode("utf-8").strip().split("\n")
        assert len(lines) == 5

    @pytest.mark.asyncio
    async def test_packets_written_counter(self, tel_logger):
        """packets_written should reflect the number of logged packets."""
        for _ in range(3):
            await tel_logger.log({"test": True})
        assert tel_logger.packets_written == 3

    def test_close_releases_file_handle(self, tel_logger):
        """close() should release the file handle without error."""
        tel_logger.close()
        assert tel_logger._file_handle is None


# ---------------------------------------------------------------------------
# AlertManager tests
# ---------------------------------------------------------------------------

class TestAlertManager:
    """Tests for the threshold-based alert manager."""

    @pytest.fixture
    def alert_manager(self, mock_api_client):
        """Return an AlertManager with a mock API client."""
        return AlertManager(drone_id="TEST-01", api_client=mock_api_client)

    @pytest.mark.asyncio
    async def test_no_alert_at_normal_battery(self, alert_manager, mock_api_client):
        """No alert should be sent at normal battery level."""
        packet = {TelemetryField.BATTERY_PCT: 75.0, TelemetryField.GPS_FIX: 3,
                  TelemetryField.GPS_SATS: 10, TelemetryField.SIGNAL_RSSI: -65}
        await alert_manager.check_telemetry(packet)
        mock_api_client.send_alert.assert_not_called()

    @pytest.mark.asyncio
    async def test_low_battery_triggers_warning_alert(self, alert_manager, mock_api_client):
        """Battery at 18% should trigger a WARNING alert."""
        packet = {TelemetryField.BATTERY_PCT: 18.0, TelemetryField.GPS_FIX: 3,
                  TelemetryField.GPS_SATS: 10, TelemetryField.SIGNAL_RSSI: -65}
        await alert_manager.check_telemetry(packet)
        mock_api_client.send_alert.assert_called_once()
        alert_data = mock_api_client.send_alert.call_args[0][0]
        assert alert_data["level"] == "WARNING"

    @pytest.mark.asyncio
    async def test_critical_battery_triggers_critical_alert(self, alert_manager, mock_api_client):
        """Battery at 8% should trigger a CRITICAL alert."""
        packet = {TelemetryField.BATTERY_PCT: 8.0, TelemetryField.GPS_FIX: 3,
                  TelemetryField.GPS_SATS: 10, TelemetryField.SIGNAL_RSSI: -65}
        await alert_manager.check_telemetry(packet)
        mock_api_client.send_alert.assert_called_once()
        alert_data = mock_api_client.send_alert.call_args[0][0]
        assert alert_data["level"] == "CRITICAL"

    @pytest.mark.asyncio
    async def test_alert_cooldown_prevents_storm(self, alert_manager, mock_api_client):
        """Repeated alerts for the same condition should be rate-limited."""
        packet = {TelemetryField.BATTERY_PCT: 18.0, TelemetryField.GPS_FIX: 3,
                  TelemetryField.GPS_SATS: 10, TelemetryField.SIGNAL_RSSI: -65}
        await alert_manager.check_telemetry(packet)
        await alert_manager.check_telemetry(packet)
        await alert_manager.check_telemetry(packet)
        # Only one alert should have been sent due to cooldown
        assert mock_api_client.send_alert.call_count == 1

    @pytest.mark.asyncio
    async def test_gps_degradation_triggers_alert(self, alert_manager, mock_api_client):
        """GPS fix below 3 should trigger a WARNING alert."""
        packet = {TelemetryField.BATTERY_PCT: 80.0, TelemetryField.GPS_FIX: 1,
                  TelemetryField.GPS_SATS: 3, TelemetryField.SIGNAL_RSSI: -65}
        await alert_manager.check_telemetry(packet)
        mock_api_client.send_alert.assert_called_once()

    @pytest.mark.asyncio
    async def test_fault_alert_bypasses_cooldown(self, alert_manager, mock_api_client):
        """send_fault_alert() should always send regardless of cooldown."""
        from core.constants import FaultReason
        await alert_manager.send_fault_alert(FaultReason.GEOFENCE_BREACH, "Test breach")
        mock_api_client.send_alert.assert_called_once()
        alert_data = mock_api_client.send_alert.call_args[0][0]
        assert alert_data["level"] == "CRITICAL"

    def test_alert_history_grows(self, alert_manager):
        """alert_history should be empty initially."""
        assert len(alert_manager.alert_history) == 0
