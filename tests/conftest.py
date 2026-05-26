"""
================================================================================
Project:    River Horizon — Autonomous Drone Fleet Management System
File:       tests/conftest.py
Purpose:    Shared pytest fixtures available to all test modules.
Author:     [Author Placeholder]
Version:    1.0.0
Date:       2026-05-25
Part of:    River Song AI Ecosystem (riversongai.com)
================================================================================
"""

import asyncio
import pytest
from unittest.mock import AsyncMock, MagicMock

from flight.mavlink_bridge import VehicleState


# ---------------------------------------------------------------------------
# Event loop
# ---------------------------------------------------------------------------

@pytest.fixture(scope="session")
def event_loop_policy():
    """Use the default asyncio event loop policy."""
    return asyncio.DefaultEventLoopPolicy()


# ---------------------------------------------------------------------------
# Shared vehicle state snapshot
# ---------------------------------------------------------------------------

GOOD_STATE = {
    "latitude": 51.5074,
    "longitude": -0.1278,
    "altitude_m": 30.0,
    "relative_altitude_m": 30.0,
    "heading_deg": 90.0,
    "speed_ms": 5.0,
    "battery_pct": 80.0,
    "battery_voltage": 12.4,
    "flight_mode": "GUIDED",
    "armed": True,
    "gps_fix": 3,
    "gps_sats": 10,
    "system_status": "OK",
    "last_heartbeat": 1000.0,
}


@pytest.fixture
def good_state():
    """Return a copy of the standard good vehicle state dict."""
    return dict(GOOD_STATE)


@pytest.fixture
def mock_bridge(good_state):
    """
    Shared mock MAVLink bridge fixture.
    Returns a MagicMock with vehicle_state.snapshot pre-configured.
    """
    bridge = MagicMock()
    bridge._drone_id = "TEST-01"
    bridge.vehicle_state = MagicMock(spec=VehicleState)
    bridge.vehicle_state.snapshot = AsyncMock(return_value=good_state)
    bridge.vehicle_state.armed = True
    bridge.vehicle_state.gps_fix = 3
    bridge.vehicle_state.battery_pct = 80.0
    bridge.vehicle_state.last_heartbeat = 1000.0
    bridge.set_mode = AsyncMock(return_value=True)
    bridge.arm = AsyncMock(return_value=True)
    bridge.disarm = AsyncMock(return_value=True)
    bridge.send_command_long = AsyncMock(return_value=True)
    bridge.is_connected = True
    bridge.register_handler = MagicMock()
    return bridge


@pytest.fixture
def mock_fault_manager():
    """Shared mock FaultManager fixture."""
    fm = MagicMock()
    fm.report_fault = AsyncMock()
    fm.resolve_fault = AsyncMock()
    fm.has_active_faults = False
    return fm


@pytest.fixture
def mock_api_client():
    """Shared mock RiverSongAPIClient fixture."""
    client = MagicMock()
    client.push_telemetry = AsyncMock(return_value=True)
    client.send_alert = AsyncMock(return_value=True)
    client.poll_commands = AsyncMock(return_value=None)
    client.report_status = AsyncMock(return_value=True)
    client.register_drone = AsyncMock(return_value=True)
    return client
