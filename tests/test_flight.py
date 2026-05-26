"""
================================================================================
Project:    River Horizon — Autonomous Drone Fleet Management System
File:       tests/test_flight.py
Purpose:    Unit tests for flight subsystems: MAVLink bridge, mode manager,
            flight controller, waypoint manager, and return-to-home controller.
Author:     [Author Placeholder]
Version:    1.0.0
Date:       2026-05-25
Part of:    River Song AI Ecosystem (riversongai.com)
================================================================================
"""

import asyncio
import pytest
from unittest.mock import AsyncMock, MagicMock, patch

from core.constants import FlightMode, FaultReason
from flight.mavlink_bridge import MAVLinkBridge, VehicleState
from flight.mode_manager import ModeManager, ModeTransitionError
from flight.flight_controller import FlightController, FlightCommandError
from flight.waypoint_manager import WaypointManager, Waypoint, Mission
from flight.return_home import ReturnHomeController


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def vehicle_state():
    """Return a VehicleState with safe default values."""
    state = VehicleState()
    state.armed = True
    state.gps_fix = 3
    state.gps_sats = 10
    state.battery_pct = 80.0
    state.battery_voltage = 12.4
    state.relative_altitude_m = 0.0
    state.latitude = 51.5074
    state.longitude = -0.1278
    state.last_heartbeat = asyncio.get_event_loop().time()
    return state


@pytest.fixture
def mock_bridge(vehicle_state):
    """Return a mock MAVLinkBridge with a pre-populated VehicleState."""
    bridge = MagicMock(spec=MAVLinkBridge)
    bridge._drone_id = "TEST-01"
    bridge.vehicle_state = vehicle_state
    bridge.vehicle_state.snapshot = AsyncMock(return_value={
        "latitude": 51.5074,
        "longitude": -0.1278,
        "altitude_m": 0.0,
        "relative_altitude_m": 0.0,
        "heading_deg": 0.0,
        "speed_ms": 0.0,
        "battery_pct": 80.0,
        "battery_voltage": 12.4,
        "flight_mode": "STABILIZE",
        "armed": True,
        "gps_fix": 3,
        "gps_sats": 10,
        "system_status": "OK",
        "last_heartbeat": 1000.0,
    })
    bridge.set_mode = AsyncMock(return_value=True)
    bridge.arm = AsyncMock(return_value=True)
    bridge.disarm = AsyncMock(return_value=True)
    bridge.send_command_long = AsyncMock(return_value=True)
    bridge.is_connected = True
    return bridge


@pytest.fixture
def mock_fault_manager():
    """Return a mock FaultManager."""
    fm = MagicMock()
    fm.report_fault = AsyncMock()
    return fm


@pytest.fixture
def mock_geofence():
    """Return a mock GeofenceMonitor that always allows positions."""
    gf = MagicMock()
    gf.is_position_allowed = AsyncMock(return_value=True)
    return gf


@pytest.fixture
def mode_manager(mock_bridge):
    """Return an initialised ModeManager."""
    return ModeManager(bridge=mock_bridge, drone_id="TEST-01")


@pytest.fixture
def flight_controller(mock_bridge, mode_manager, mock_geofence, mock_fault_manager):
    """Return an initialised FlightController."""
    return FlightController(
        bridge=mock_bridge,
        mode_manager=mode_manager,
        geofence=mock_geofence,
        fault_manager=mock_fault_manager,
        max_altitude_m=120.0,
        max_speed_ms=15.0,
    )


# ---------------------------------------------------------------------------
# VehicleState tests
# ---------------------------------------------------------------------------

class TestVehicleState:
    """Tests for the VehicleState data container."""

    @pytest.mark.asyncio
    async def test_update_known_field(self):
        """Updating a known field should succeed."""
        state = VehicleState()
        await state.update(battery_pct=55.0)
        assert state.battery_pct == 55.0

    @pytest.mark.asyncio
    async def test_update_unknown_field_is_ignored(self):
        """Updating an unknown field should log a warning but not raise."""
        state = VehicleState()
        await state.update(nonexistent_field=42)  # Should not raise

    @pytest.mark.asyncio
    async def test_snapshot_returns_dict(self):
        """snapshot() should return a dictionary with all expected keys."""
        state = VehicleState()
        snap = await state.snapshot()
        assert isinstance(snap, dict)
        assert "latitude" in snap
        assert "battery_pct" in snap
        assert "armed" in snap


# ---------------------------------------------------------------------------
# ModeManager tests
# ---------------------------------------------------------------------------

class TestModeManager:
    """Tests for flight mode management."""

    @pytest.mark.asyncio
    async def test_initialise_registers_handler(self, mode_manager, mock_bridge):
        """initialise() should register a HEARTBEAT handler."""
        await mode_manager.initialise()
        mock_bridge.register_handler.assert_called_with("HEARTBEAT", mode_manager._on_heartbeat)

    @pytest.mark.asyncio
    async def test_request_mode_success(self, mode_manager, mock_bridge):
        """Requesting a valid mode should call bridge.set_mode."""
        await mode_manager.initialise()
        result = await mode_manager.request_mode(FlightMode.GUIDED)
        assert result is True
        mock_bridge.set_mode.assert_called_once_with(FlightMode.GUIDED.value)

    @pytest.mark.asyncio
    async def test_request_same_mode_is_noop(self, mode_manager, mock_bridge):
        """Requesting the current mode should return True without calling set_mode."""
        await mode_manager.initialise()
        # Default mode is STABILIZE
        result = await mode_manager.request_mode(FlightMode.STABILIZE)
        assert result is True
        mock_bridge.set_mode.assert_not_called()

    @pytest.mark.asyncio
    async def test_force_rtl_bypasses_checks(self, mode_manager, mock_bridge):
        """force_rtl() should succeed even if vehicle is not armed."""
        await mode_manager.initialise()
        mock_bridge.vehicle_state.armed = False
        result = await mode_manager.force_rtl()
        assert result is True

    @pytest.mark.asyncio
    async def test_is_autonomous_false_by_default(self, mode_manager):
        """is_autonomous should be False in STABILIZE mode."""
        await mode_manager.initialise()
        assert mode_manager.is_autonomous is False

    @pytest.mark.asyncio
    async def test_is_returning_after_rtl(self, mode_manager, mock_bridge):
        """is_returning should be True after RTL mode is set."""
        await mode_manager.initialise()
        await mode_manager.force_rtl()
        assert mode_manager.is_returning is True


# ---------------------------------------------------------------------------
# FlightController tests
# ---------------------------------------------------------------------------

class TestFlightController:
    """Tests for the high-level flight controller."""

    @pytest.mark.asyncio
    async def test_arm_succeeds_with_good_state(self, flight_controller, mock_bridge):
        """arm() should succeed when GPS fix and battery are adequate."""
        await flight_controller.initialise()
        result = await flight_controller.arm()
        assert result is True
        mock_bridge.arm.assert_called_once()

    @pytest.mark.asyncio
    async def test_arm_fails_with_low_battery(self, flight_controller, mock_bridge):
        """arm() should raise FlightCommandError when battery is below 30%."""
        await flight_controller.initialise()
        mock_bridge.vehicle_state.snapshot = AsyncMock(return_value={
            "gps_fix": 3, "battery_pct": 25.0, "relative_altitude_m": 0.0,
            "armed": False, "latitude": 0.0, "longitude": 0.0,
            "altitude_m": 0.0, "heading_deg": 0.0, "speed_ms": 0.0,
            "battery_voltage": 11.0, "flight_mode": "STABILIZE",
            "gps_sats": 8, "system_status": "OK", "last_heartbeat": 1000.0,
        })
        with pytest.raises(FlightCommandError, match="Battery too low"):
            await flight_controller.arm()

    @pytest.mark.asyncio
    async def test_arm_fails_with_poor_gps(self, flight_controller, mock_bridge):
        """arm() should raise FlightCommandError when GPS fix is insufficient."""
        await flight_controller.initialise()
        mock_bridge.vehicle_state.snapshot = AsyncMock(return_value={
            "gps_fix": 1, "battery_pct": 80.0, "relative_altitude_m": 0.0,
            "armed": False, "latitude": 0.0, "longitude": 0.0,
            "altitude_m": 0.0, "heading_deg": 0.0, "speed_ms": 0.0,
            "battery_voltage": 12.4, "flight_mode": "STABILIZE",
            "gps_sats": 3, "system_status": "OK", "last_heartbeat": 1000.0,
        })
        with pytest.raises(FlightCommandError, match="GPS fix"):
            await flight_controller.arm()

    @pytest.mark.asyncio
    async def test_altitude_is_clamped(self, flight_controller):
        """Altitude above max should be clamped to max_altitude_m."""
        clamped = flight_controller._clamp_altitude(200.0)
        assert clamped == 120.0

    @pytest.mark.asyncio
    async def test_goto_position_blocked_by_geofence(
        self, flight_controller, mock_geofence
    ):
        """goto_position should raise FlightCommandError on geofence violation."""
        mock_geofence.is_position_allowed = AsyncMock(return_value=False)
        with pytest.raises(FlightCommandError, match="geofence"):
            await flight_controller.goto_position(99.0, 99.0, 50.0)


# ---------------------------------------------------------------------------
# WaypointManager tests
# ---------------------------------------------------------------------------

class TestWaypointManager:
    """Tests for waypoint and mission management."""

    def _make_wm(self, mock_bridge, flight_controller):
        """Helper to create a WaypointManager."""
        return WaypointManager(
            bridge=mock_bridge,
            flight_controller=flight_controller,
            waypoint_radius_m=2.0,
        )

    def test_register_and_retrieve_waypoint(self, mock_bridge, flight_controller):
        """Registered waypoints should be retrievable by name."""
        wm = self._make_wm(mock_bridge, flight_controller)
        wp = Waypoint(name="Alpha", latitude=51.5, longitude=-0.1, altitude_m=30.0)
        wm.register_waypoint(wp)
        assert wm.get_waypoint("alpha") == wp
        assert wm.get_waypoint("ALPHA") == wp  # Case-insensitive

    def test_get_unknown_waypoint_returns_none(self, mock_bridge, flight_controller):
        """Retrieving an unregistered waypoint should return None."""
        wm = self._make_wm(mock_bridge, flight_controller)
        assert wm.get_waypoint("nonexistent") is None

    def test_list_waypoints_sorted(self, mock_bridge, flight_controller):
        """list_waypoints() should return sorted names."""
        wm = self._make_wm(mock_bridge, flight_controller)
        for name in ["Charlie", "Alpha", "Bravo"]:
            wm.register_waypoint(Waypoint(name=name, latitude=0, longitude=0, altitude_m=10))
        assert wm.list_waypoints() == ["alpha", "bravo", "charlie"]

    @pytest.mark.asyncio
    async def test_goto_named_waypoint_raises_on_unknown(self, mock_bridge, flight_controller):
        """goto_named_waypoint should raise KeyError for unknown waypoints."""
        wm = self._make_wm(mock_bridge, flight_controller)
        with pytest.raises(KeyError):
            await wm.goto_named_waypoint("unknown")

    def test_haversine_distance_same_point(self, mock_bridge, flight_controller):
        """Haversine distance between identical points should be 0."""
        wm = self._make_wm(mock_bridge, flight_controller)
        dist = wm._haversine_distance(51.5, -0.1, 51.5, -0.1)
        assert dist == pytest.approx(0.0, abs=0.01)

    def test_haversine_distance_known_value(self, mock_bridge, flight_controller):
        """Haversine distance between two known points should be approximately correct."""
        wm = self._make_wm(mock_bridge, flight_controller)
        # London to Paris ≈ 340 km
        dist = wm._haversine_distance(51.5074, -0.1278, 48.8566, 2.3522)
        assert 330_000 < dist < 350_000


# ---------------------------------------------------------------------------
# ReturnHomeController tests
# ---------------------------------------------------------------------------

class TestReturnHomeController:
    """Tests for the return-to-home controller."""

    def _make_rth(self, mock_bridge, mode_manager, mock_fault_manager):
        """Helper to create a ReturnHomeController."""
        return ReturnHomeController(
            bridge=mock_bridge,
            mode_manager=mode_manager,
            rth_altitude_m=50.0,
            fault_manager=mock_fault_manager,
        )

    @pytest.mark.asyncio
    async def test_rth_not_active_initially(self, mock_bridge, mode_manager, mock_fault_manager):
        """RTH should not be active on initialisation."""
        rth = self._make_rth(mock_bridge, mode_manager, mock_fault_manager)
        assert rth.is_rth_active is False

    @pytest.mark.asyncio
    async def test_execute_rth_when_not_armed(self, mock_bridge, mode_manager, mock_fault_manager):
        """RTH on an unarmed vehicle should be a no-op."""
        mock_bridge.vehicle_state.snapshot = AsyncMock(return_value={
            "armed": False, "relative_altitude_m": 0.0,
            "latitude": 0.0, "longitude": 0.0, "altitude_m": 0.0,
            "heading_deg": 0.0, "speed_ms": 0.0, "battery_pct": 80.0,
            "battery_voltage": 12.4, "flight_mode": "STABILIZE",
            "gps_fix": 3, "gps_sats": 10, "system_status": "OK",
            "last_heartbeat": 1000.0,
        })
        rth = self._make_rth(mock_bridge, mode_manager, mock_fault_manager)
        await mode_manager.initialise()
        result = await rth.execute_rth(FaultReason.LOW_BATTERY)
        assert result is True
        assert rth.is_rth_active is False  # Cleared after no-op

    @pytest.mark.asyncio
    async def test_duplicate_rth_is_ignored(self, mock_bridge, mode_manager, mock_fault_manager):
        """A second RTH trigger while RTH is active should be ignored."""
        mock_bridge.vehicle_state.snapshot = AsyncMock(return_value={
            "armed": True, "relative_altitude_m": 60.0,
            "latitude": 51.5, "longitude": -0.1, "altitude_m": 60.0,
            "heading_deg": 0.0, "speed_ms": 5.0, "battery_pct": 18.0,
            "battery_voltage": 11.0, "flight_mode": "GUIDED",
            "gps_fix": 3, "gps_sats": 10, "system_status": "OK",
            "last_heartbeat": 1000.0,
        })
        rth = self._make_rth(mock_bridge, mode_manager, mock_fault_manager)
        await mode_manager.initialise()
        await rth.execute_rth(FaultReason.LOW_BATTERY)
        # Second call should return True without re-triggering
        result = await rth.execute_rth(FaultReason.SIGNAL_LOSS)
        assert result is True
