"""
================================================================================
Project:    River Horizon — Autonomous Drone Fleet Management System
File:       tests/test_safety.py
Purpose:    Unit tests for safety subsystems: geofence, battery monitor,
            signal watchdog, and fault manager.
Author:     [Author Placeholder]
Version:    1.0.0
Date:       2026-05-25
Part of:    River Song AI Ecosystem (riversongai.com)
================================================================================
"""

import asyncio
import pytest
from unittest.mock import AsyncMock, MagicMock, patch

from core.constants import FaultReason, SystemStatus
from safety.geofence import GeofenceMonitor
from safety.battery_monitor import BatteryMonitor
from safety.signal_watchdog import SignalWatchdog
from safety.fault_manager import FaultManager, FaultRecord


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def mock_fault_manager():
    """Return a mock FaultManager."""
    fm = MagicMock(spec=FaultManager)
    fm.report_fault = AsyncMock()
    return fm


@pytest.fixture
def mock_bridge():
    """Return a mock MAVLink bridge."""
    bridge = MagicMock()
    bridge.vehicle_state = MagicMock()
    bridge.vehicle_state.snapshot = AsyncMock(return_value={
        "latitude": 51.5074,
        "longitude": -0.1278,
        "altitude_m": 30.0,
        "relative_altitude_m": 30.0,
        "heading_deg": 0.0,
        "speed_ms": 5.0,
        "battery_pct": 80.0,
        "battery_voltage": 12.4,
        "flight_mode": "GUIDED",
        "armed": True,
        "gps_fix": 3,
        "gps_sats": 10,
        "system_status": "OK",
        "last_heartbeat": 1000.0,
    })
    return bridge


# ---------------------------------------------------------------------------
# GeofenceMonitor tests
# ---------------------------------------------------------------------------

class TestGeofenceMonitor:
    """Tests for the cylindrical geofence enforcer."""

    @pytest.fixture
    def geofence(self, mock_fault_manager):
        """Return an initialised GeofenceMonitor."""
        gf = GeofenceMonitor(
            drone_id="TEST-01",
            radius_m=500.0,
            max_altitude_m=120.0,
            fault_manager=mock_fault_manager,
        )
        gf.set_home(51.5074, -0.1278, 0.0)
        return gf

    @pytest.mark.asyncio
    async def test_initialise_sets_ok_status(self, mock_fault_manager):
        """initialise() should set status to OK."""
        gf = GeofenceMonitor("TEST-01", 500.0, 120.0, mock_fault_manager)
        await gf.initialise()
        assert gf.status == SystemStatus.OK

    @pytest.mark.asyncio
    async def test_position_inside_geofence_is_allowed(self, geofence):
        """A position within the radius and altitude should be allowed."""
        # 100m north of home — well within 500m radius
        result = await geofence.is_position_allowed(51.5083, -0.1278, 50.0)
        assert result is True

    @pytest.mark.asyncio
    async def test_position_outside_radius_is_denied(self, geofence):
        """A position beyond the radius should be denied."""
        # ~1km north of home — outside 500m radius
        result = await geofence.is_position_allowed(51.5164, -0.1278, 50.0)
        assert result is False

    @pytest.mark.asyncio
    async def test_position_above_ceiling_is_denied(self, geofence):
        """A position above the altitude ceiling should be denied."""
        result = await geofence.is_position_allowed(51.5074, -0.1278, 150.0)
        assert result is False

    @pytest.mark.asyncio
    async def test_no_home_set_allows_all_positions(self, mock_fault_manager):
        """Without a home position set, all positions should be allowed."""
        gf = GeofenceMonitor("TEST-01", 500.0, 120.0, mock_fault_manager)
        await gf.initialise()
        result = await gf.is_position_allowed(0.0, 0.0, 200.0)
        assert result is True

    def test_haversine_distance_accuracy(self, geofence):
        """Haversine should return ~111m for 0.001 degree latitude difference."""
        dist = geofence._haversine_distance(51.5074, -0.1278, 51.5084, -0.1278)
        assert 100 < dist < 120

    @pytest.mark.asyncio
    async def test_breach_triggers_fault(self, geofence, mock_fault_manager, mock_bridge):
        """A position breach should call fault_manager.report_fault."""
        geofence.set_bridge(mock_bridge)
        # Simulate position outside geofence
        mock_bridge.vehicle_state.snapshot = AsyncMock(return_value={
            "latitude": 51.5164,    # ~1km north — outside 500m
            "longitude": -0.1278,
            "relative_altitude_m": 30.0,
            "gps_fix": 3,
        })
        await geofence._check_position()
        mock_fault_manager.report_fault.assert_called_once_with(
            FaultReason.GEOFENCE_BREACH,
            pytest.approx(mock_fault_manager.report_fault.call_args[0][1], rel=0.1),
        )

    @pytest.mark.asyncio
    async def test_no_breach_when_inside(self, geofence, mock_fault_manager, mock_bridge):
        """No fault should be reported when the drone is inside the geofence."""
        geofence.set_bridge(mock_bridge)
        await geofence._check_position()
        mock_fault_manager.report_fault.assert_not_called()


# ---------------------------------------------------------------------------
# BatteryMonitor tests
# ---------------------------------------------------------------------------

class TestBatteryMonitor:
    """Tests for the battery health monitor."""

    @pytest.fixture
    def battery_monitor(self, mock_fault_manager):
        """Return an initialised BatteryMonitor."""
        bm = BatteryMonitor(
            drone_id="TEST-01",
            low_pct=20.0,
            critical_pct=10.0,
            fault_manager=mock_fault_manager,
        )
        return bm

    @pytest.mark.asyncio
    async def test_initialise_sets_ok_status(self, battery_monitor):
        """initialise() should set status to OK."""
        await battery_monitor.initialise()
        assert battery_monitor.status == SystemStatus.OK

    @pytest.mark.asyncio
    async def test_no_fault_at_normal_battery(self, battery_monitor, mock_fault_manager, mock_bridge):
        """No fault should be reported at normal battery level."""
        await battery_monitor.initialise()
        battery_monitor.set_bridge(mock_bridge)
        await battery_monitor._check_battery()
        mock_fault_manager.report_fault.assert_not_called()

    @pytest.mark.asyncio
    async def test_low_battery_triggers_rth_fault(self, battery_monitor, mock_fault_manager, mock_bridge):
        """Battery at low threshold should trigger LOW_BATTERY fault."""
        await battery_monitor.initialise()
        battery_monitor.set_bridge(mock_bridge)
        mock_bridge.vehicle_state.snapshot = AsyncMock(return_value={
            "battery_pct": 18.0, "battery_voltage": 11.0,
        })
        await battery_monitor._check_battery()
        mock_fault_manager.report_fault.assert_called_once()
        args = mock_fault_manager.report_fault.call_args[0]
        assert args[0] == FaultReason.LOW_BATTERY

    @pytest.mark.asyncio
    async def test_critical_battery_triggers_land_fault(self, battery_monitor, mock_fault_manager, mock_bridge):
        """Battery at critical threshold should trigger CRITICAL_BATTERY fault."""
        await battery_monitor.initialise()
        battery_monitor.set_bridge(mock_bridge)
        mock_bridge.vehicle_state.snapshot = AsyncMock(return_value={
            "battery_pct": 8.0, "battery_voltage": 10.5,
        })
        await battery_monitor._check_battery()
        mock_fault_manager.report_fault.assert_called_once()
        args = mock_fault_manager.report_fault.call_args[0]
        assert args[0] == FaultReason.CRITICAL_BATTERY

    @pytest.mark.asyncio
    async def test_low_battery_not_triggered_twice(self, battery_monitor, mock_fault_manager, mock_bridge):
        """LOW_BATTERY fault should only be reported once per event."""
        await battery_monitor.initialise()
        battery_monitor.set_bridge(mock_bridge)
        mock_bridge.vehicle_state.snapshot = AsyncMock(return_value={
            "battery_pct": 15.0, "battery_voltage": 11.2,
        })
        await battery_monitor._check_battery()
        await battery_monitor._check_battery()
        assert mock_fault_manager.report_fault.call_count == 1

    def test_update_thresholds(self, battery_monitor):
        """update_thresholds() should update the stored values."""
        battery_monitor.update_thresholds(25.0, 12.0)
        assert battery_monitor._low_pct == 25.0
        assert battery_monitor._critical_pct == 12.0


# ---------------------------------------------------------------------------
# FaultManager tests
# ---------------------------------------------------------------------------

class TestFaultManager:
    """Tests for the central fault aggregator."""

    @pytest.fixture
    def fault_manager(self):
        """Return a FaultManager with mock handlers."""
        fm = FaultManager(drone_id="TEST-01")
        fm._rth_handler = AsyncMock()
        fm._land_handler = AsyncMock()
        return fm

    @pytest.mark.asyncio
    async def test_report_fault_calls_rth_handler(self, fault_manager):
        """Reporting a LOW_BATTERY fault should call the RTH handler."""
        await fault_manager.report_fault(FaultReason.LOW_BATTERY, "Test low battery")
        fault_manager._rth_handler.assert_called_once_with(FaultReason.LOW_BATTERY)

    @pytest.mark.asyncio
    async def test_report_critical_fault_calls_land_handler(self, fault_manager):
        """Reporting a CRITICAL_BATTERY fault should call the LAND handler."""
        await fault_manager.report_fault(FaultReason.CRITICAL_BATTERY, "Test critical battery")
        fault_manager._land_handler.assert_called_once()

    @pytest.mark.asyncio
    async def test_duplicate_fault_not_re_triggered(self, fault_manager):
        """Reporting the same fault twice should only trigger the handler once."""
        await fault_manager.report_fault(FaultReason.SIGNAL_LOSS, "First")
        await fault_manager.report_fault(FaultReason.SIGNAL_LOSS, "Duplicate")
        assert fault_manager._rth_handler.call_count == 1

    @pytest.mark.asyncio
    async def test_resolve_fault_removes_from_active(self, fault_manager):
        """Resolving a fault should remove it from active_faults."""
        await fault_manager.report_fault(FaultReason.GEOFENCE_BREACH, "Test breach")
        assert FaultReason.GEOFENCE_BREACH in fault_manager.active_faults
        await fault_manager.resolve_fault(FaultReason.GEOFENCE_BREACH)
        assert FaultReason.GEOFENCE_BREACH not in fault_manager.active_faults

    @pytest.mark.asyncio
    async def test_overall_status_ok_when_no_faults(self, fault_manager):
        """Overall status should be OK when no faults are active."""
        assert fault_manager.overall_status == SystemStatus.OK

    @pytest.mark.asyncio
    async def test_overall_status_failsafe_on_signal_loss(self, fault_manager):
        """Overall status should be FAILSAFE on signal loss fault."""
        await fault_manager.report_fault(FaultReason.SIGNAL_LOSS, "Test")
        assert fault_manager.overall_status == SystemStatus.FAILSAFE

    @pytest.mark.asyncio
    async def test_fault_log_grows_with_each_report(self, fault_manager):
        """Each unique fault report should add an entry to the fault log."""
        await fault_manager.report_fault(FaultReason.LOW_BATTERY, "Test 1")
        await fault_manager.report_fault(FaultReason.SIGNAL_LOSS, "Test 2")
        assert len(fault_manager.fault_log) == 2
