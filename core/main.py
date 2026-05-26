"""
================================================================================
Project:    River Horizon — Autonomous Drone Fleet Management System
File:       core/main.py
Purpose:    Application entry point. Bootstraps all subsystems in the correct
            safety-first order, starts the async event loop, and coordinates
            the main control cycle. Safety systems are always initialised
            before flight or connectivity systems.
Author:     [Author Placeholder]
Version:    1.0.0
Date:       2026-05-25
Part of:    River Song AI Ecosystem (riversongai.com)
================================================================================
"""

import argparse
import asyncio
import logging
import logging.handlers
import os
import signal
import sys
from pathlib import Path
from typing import Optional

# ---------------------------------------------------------------------------
# Bootstrap logging before any other import so all modules get the handler
# ---------------------------------------------------------------------------
from core.constants import LOG_FORMAT, LOG_DATE_FORMAT, LOG_MAX_BYTES, LOG_BACKUP_COUNT

_log_dir = Path(__file__).resolve().parent.parent / "logs"
_log_dir.mkdir(exist_ok=True)

_root_logger = logging.getLogger()
_root_logger.setLevel(logging.DEBUG)

_console_handler = logging.StreamHandler(sys.stdout)
_console_handler.setLevel(logging.INFO)
_console_handler.setFormatter(logging.Formatter(LOG_FORMAT, datefmt=LOG_DATE_FORMAT))

_file_handler = logging.handlers.RotatingFileHandler(
    _log_dir / "river_horizon.log",
    maxBytes=LOG_MAX_BYTES,
    backupCount=LOG_BACKUP_COUNT,
    encoding="utf-8",
)
_file_handler.setLevel(logging.DEBUG)
_file_handler.setFormatter(logging.Formatter(LOG_FORMAT, datefmt=LOG_DATE_FORMAT))

_root_logger.addHandler(_console_handler)
_root_logger.addHandler(_file_handler)

logger = logging.getLogger("RiverHorizon.Main")

# ---------------------------------------------------------------------------
# Core imports (after logging is configured)
# ---------------------------------------------------------------------------
from core.config import config, ConfigError
from core.constants import SystemStatus, FlightMode

from safety.fault_manager import FaultManager
from safety.geofence import GeofenceMonitor
from safety.battery_monitor import BatteryMonitor
from safety.signal_watchdog import SignalWatchdog

from flight.mavlink_bridge import MAVLinkBridge
from flight.mode_manager import ModeManager
from flight.flight_controller import FlightController
from flight.waypoint_manager import WaypointManager
from flight.return_home import ReturnHomeController

from telemetry.collector import TelemetryCollector
from telemetry.logger import TelemetryLogger
from telemetry.alerts import AlertManager

from connectivity.cellular import CellularMonitor
from connectivity.vpn import VPNManager
from connectivity.api_client import RiverSongAPIClient
from connectivity.stream_manager import StreamManager

from vision.camera_manager import CameraManager
from vision.stream_server import StreamServer
from vision.obstacle_detect import ObstacleDetector

from remote.web_controller import WebController
from connectivity.command_poller import CommandPoller
from connectivity.meshtastic_beacon import MeshtasticBeacon
from core.fleet_manager import fleet, DroneUnit, FleetManager
from sim.sitl import SITLBackend


# ---------------------------------------------------------------------------
# Shutdown flag — set by signal handlers
# ---------------------------------------------------------------------------
_shutdown_event: asyncio.Event = asyncio.Event()


def _handle_signal(sig: signal.Signals) -> None:
    """
    Handle OS signals (SIGINT, SIGTERM) for graceful shutdown.

    Args:
        sig: The received signal.
    """
    logger.warning("Received signal %s — initiating graceful shutdown.", sig.name)
    _shutdown_event.set()


# ---------------------------------------------------------------------------
# System bootstrap
# ---------------------------------------------------------------------------

async def initialise_safety_systems(fault_manager: FaultManager) -> tuple:
    """
    Initialise all safety subsystems. These MUST be ready before any flight
    or connectivity system is started.

    Args:
        fault_manager: The central fault manager that safety monitors report to.

    Returns:
        Tuple of (GeofenceMonitor, BatteryMonitor, SignalWatchdog).

    Raises:
        RuntimeError: If any safety system fails to initialise.
    """
    logger.info("=== [1/5] Initialising safety systems ===")

    geofence = GeofenceMonitor(
        drone_id=config.drone_id,
        radius_m=config.safety.geofence_radius_m,
        max_altitude_m=config.safety.geofence_max_altitude_m,
        fault_manager=fault_manager,
    )
    await geofence.initialise()
    logger.info("  ✓ Geofence armed  (radius=%.0fm, ceiling=%.0fm)",
                config.safety.geofence_radius_m, config.safety.geofence_max_altitude_m)

    battery_monitor = BatteryMonitor(
        drone_id=config.drone_id,
        low_pct=config.safety.low_battery_pct,
        critical_pct=config.safety.critical_battery_pct,
        fault_manager=fault_manager,
    )
    await battery_monitor.initialise()
    logger.info("  ✓ Battery monitor armed  (low=%.0f%%, critical=%.0f%%)",
                config.safety.low_battery_pct, config.safety.critical_battery_pct)

    signal_watchdog = SignalWatchdog(
        drone_id=config.drone_id,
        timeout_s=config.safety.signal_timeout_s,
        min_rssi_dbm=config.safety.min_rssi_dbm,
        fault_manager=fault_manager,
    )
    await signal_watchdog.initialise()
    logger.info("  ✓ Signal watchdog armed  (timeout=%.1fs, min_rssi=%ddBm)",
                config.safety.signal_timeout_s, config.safety.min_rssi_dbm)

    return geofence, battery_monitor, signal_watchdog


async def initialise_flight_systems(
    fault_manager: FaultManager,
    geofence: GeofenceMonitor,
) -> tuple:
    """
    Initialise MAVLink bridge and all flight subsystems.

    Args:
        fault_manager: Central fault manager.
        geofence:      Active geofence monitor (passed to flight controller).

    Returns:
        Tuple of (MAVLinkBridge, ModeManager, FlightController,
                  WaypointManager, ReturnHomeController).

    Raises:
        RuntimeError: If MAVLink connection cannot be established.
    """
    logger.info("=== [2/5] Initialising flight systems ===")

    bridge = MAVLinkBridge(
        connection_string=config.connectivity.mavlink_connection_string,
        baud_rate=config.connectivity.baud_rate,
        drone_id=config.drone_id,
    )
    await bridge.connect()
    logger.info("  ✓ MAVLink bridge connected  (%s @ %d baud)",
                config.connectivity.mavlink_connection_string,
                config.connectivity.baud_rate)

    mode_manager = ModeManager(bridge=bridge, drone_id=config.drone_id)
    await mode_manager.initialise()
    logger.info("  ✓ Mode manager ready")

    flight_controller = FlightController(
        bridge=bridge,
        mode_manager=mode_manager,
        geofence=geofence,
        fault_manager=fault_manager,
        max_altitude_m=config.flight.max_altitude_m,
        max_speed_ms=config.flight.max_speed_ms,
    )
    await flight_controller.initialise()
    logger.info("  ✓ Flight controller ready")

    waypoint_manager = WaypointManager(
        bridge=bridge,
        flight_controller=flight_controller,
        waypoint_radius_m=config.flight.waypoint_radius_m,
    )
    logger.info("  ✓ Waypoint manager ready")

    rth_controller = ReturnHomeController(
        bridge=bridge,
        mode_manager=mode_manager,
        rth_altitude_m=config.flight.rth_altitude_m,
        fault_manager=fault_manager,
    )
    logger.info("  ✓ Return-to-home controller ready")

    return bridge, mode_manager, flight_controller, waypoint_manager, rth_controller


async def initialise_connectivity(fault_manager: FaultManager) -> tuple:
    """
    Initialise cellular, VPN, River Song API client, and stream manager.

    Args:
        fault_manager: Central fault manager.

    Returns:
        Tuple of (CellularMonitor, VPNManager, RiverSongAPIClient, StreamManager).
    """
    logger.info("=== [3/5] Initialising connectivity ===")

    cellular = CellularMonitor(
        interface=config.connectivity.cellular_interface,
        fault_manager=fault_manager,
    )
    await cellular.initialise()
    logger.info("  ✓ Cellular monitor active  (iface=%s)", config.connectivity.cellular_interface)

    vpn = VPNManager(
        enabled=config.connectivity.vpn_enabled,
        interface=config.connectivity.vpn_interface,
        config_path=config.connectivity.vpn_config_path,
    )
    await vpn.initialise()
    logger.info("  ✓ WireGuard VPN  (enabled=%s, iface=%s)",
                config.connectivity.vpn_enabled, config.connectivity.vpn_interface)

    api_client = RiverSongAPIClient(
        base_url=config.river_song.base_url,
        api_key=config.river_song.api_key,
        drone_id=config.drone_id,
    )
    logger.info("  ✓ River Song API client ready  (url=%s)", config.river_song.base_url)

    stream_manager = StreamManager(drone_id=config.drone_id)
    logger.info("  ✓ Stream manager ready")

    return cellular, vpn, api_client, stream_manager


async def initialise_vision(stream_manager: StreamManager) -> tuple:
    """
    Initialise camera, MJPEG/RTSP stream server, and obstacle detector.

    Args:
        stream_manager: Stream manager for registering active streams.

    Returns:
        Tuple of (CameraManager, StreamServer, ObstacleDetector).
    """
    logger.info("=== [4/5] Initialising vision systems ===")

    camera = CameraManager(
        camera_index=config.streaming.camera_index,
        width=config.streaming.width,
        height=config.streaming.height,
        fps=config.streaming.fps,
    )
    await camera.initialise()
    logger.info("  ✓ Camera manager ready  (%dx%d @ %dfps)",
                config.streaming.width, config.streaming.height, config.streaming.fps)

    stream_server = StreamServer(
        camera_manager=camera,
        port=config.streaming.port,
        on_demand=config.streaming.on_demand,
        stream_manager=stream_manager,
    )
    await stream_server.initialise()
    logger.info("  ✓ Stream server ready  (port=%d, on_demand=%s)",
                config.streaming.port, config.streaming.on_demand)

    obstacle_detector = ObstacleDetector(camera_manager=camera)
    logger.info("  ✓ Obstacle detector ready")

    return camera, stream_server, obstacle_detector


async def initialise_telemetry(
    bridge: MAVLinkBridge,
    api_client: RiverSongAPIClient,
) -> tuple:
    """
    Initialise telemetry collector, structured logger, and alert manager.

    Args:
        bridge:     MAVLink bridge for reading vehicle state.
        api_client: River Song API client for pushing telemetry upstream.

    Returns:
        Tuple of (TelemetryCollector, TelemetryLogger, AlertManager).
    """
    logger.info("=== [5/5] Initialising telemetry ===")

    tel_logger = TelemetryLogger(drone_id=config.drone_id, log_dir=_log_dir / "telemetry")
    logger.info("  ✓ Telemetry logger ready")

    alert_manager = AlertManager(drone_id=config.drone_id, api_client=api_client)
    logger.info("  ✓ Alert manager ready")

    collector = TelemetryCollector(
        bridge=bridge,
        drone_id=config.drone_id,
        tel_logger=tel_logger,
        alert_manager=alert_manager,
        api_client=api_client,
        rate_hz=10,
    )
    logger.info("  ✓ Telemetry collector ready  (rate=10 Hz)")

    return collector, tel_logger, alert_manager


# ---------------------------------------------------------------------------
# Main async entry point
# ---------------------------------------------------------------------------

async def run() -> None:
    """
    Primary async coroutine. Bootstraps all subsystems in safety-first order,
    then runs the main control loop until a shutdown signal is received.
    """
    sitl = SITLBackend.get()
    sim_label = "  [SIMULATION MODE — no drone required]" if sitl else ""

    logger.info("=" * 72)
    logger.info("  River Horizon — Drone Fleet Management System")
    logger.info("  Drone ID : %s", config.drone_id)
    logger.info("  Model    : %s", config.model)
    logger.info("  Version  : 1.0.0")
    if sim_label:
        logger.info(sim_label)
    logger.info("=" * 72)

    # Register OS signal handlers
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, _handle_signal, sig)

    # ------------------------------------------------------------------
    # 1. Safety systems — MUST come first
    # ------------------------------------------------------------------
    fault_manager = FaultManager(drone_id=config.drone_id)
    geofence, battery_monitor, signal_watchdog = await initialise_safety_systems(fault_manager)

    # ------------------------------------------------------------------
    # 2. Flight systems
    # ------------------------------------------------------------------
    bridge, mode_manager, flight_controller, waypoint_manager, rth_controller = \
        await initialise_flight_systems(fault_manager, geofence)

    # Wire RTH into fault manager so any fault triggers RTH automatically
    fault_manager.register_rth_handler(rth_controller.execute_rth)

    # ------------------------------------------------------------------
    # 3. Connectivity
    # ------------------------------------------------------------------
    cellular, vpn, api_client, stream_manager = await initialise_connectivity(fault_manager)

    # ------------------------------------------------------------------
    # 3b. Meshtastic backup beacon
    # ------------------------------------------------------------------
    meshtastic_port = os.getenv("MESHTASTIC_PORT", "/dev/ttyUSB1")
    beacon = MeshtasticBeacon(
        drone_id=config.drone_id,
        port=meshtastic_port,
        bridge=bridge,
        fault_manager=fault_manager,
        on_rth=rth_controller.execute_rth,
        on_land=flight_controller.land,
        cellular_rssi=lambda: cellular.rssi_dbm,
        sim_mode=SITLBackend.active,
    )
    await beacon.initialise()
    logger.info("  ✓ Meshtastic beacon ready  (port=%s, sim=%s)",
                meshtastic_port, SITLBackend.active)

    # ------------------------------------------------------------------
    # 4. Vision
    # ------------------------------------------------------------------
    camera, stream_server, obstacle_detector = await initialise_vision(stream_manager)

    # ------------------------------------------------------------------
    # 5. Telemetry
    # ------------------------------------------------------------------
    collector, tel_logger, alert_manager = await initialise_telemetry(bridge, api_client)

    # ------------------------------------------------------------------
    # 6. Web / remote control interface
    # ------------------------------------------------------------------
    web_controller = WebController(
        drone_id=config.drone_id,
        flight_controller=flight_controller,
        waypoint_manager=waypoint_manager,
        mode_manager=mode_manager,
        stream_manager=stream_manager,
        telemetry_collector=collector,
        api_prefix=config.river_song.api_prefix,
    )

    # ------------------------------------------------------------------
    # 7. River Song command poller
    # ------------------------------------------------------------------
    command_poller = CommandPoller(
        drone_id=config.drone_id,
        api_client=api_client,
        flight_controller=flight_controller,
        waypoint_manager=waypoint_manager,
        mode_manager=mode_manager,
        stream_manager=stream_manager,
        fault_manager=fault_manager,
    )

    # ------------------------------------------------------------------
    # 8. Load named waypoints from drone profile
    # ------------------------------------------------------------------
    _load_named_waypoints(waypoint_manager)

    # ------------------------------------------------------------------
    # 9. Register this unit with the fleet manager
    # ------------------------------------------------------------------
    unit = DroneUnit(
        drone_id=config.drone_id,
        model=config.model,
        flight_controller=flight_controller,
        waypoint_manager=waypoint_manager,
        mode_manager=mode_manager,
        telemetry_collector=collector,
        fault_manager=fault_manager,
        stream_manager=stream_manager,
    )
    await fleet.register_unit(unit)

    # ------------------------------------------------------------------
    # 10. Register this drone with the River Song API
    # ------------------------------------------------------------------
    if config.river_song.enabled:
        await api_client.register_drone({
            "drone_id": config.drone_id,
            "model": config.model,
            "weight_kg": config.weight_kg,
            "api_prefix": config.river_song.api_prefix,
        })

    # ------------------------------------------------------------------
    # 11. Start all background tasks
    # ------------------------------------------------------------------
    logger.info("Starting background tasks...")
    tasks = [
        asyncio.create_task(geofence.run(),             name="geofence"),
        asyncio.create_task(battery_monitor.run(),      name="battery_monitor"),
        asyncio.create_task(signal_watchdog.run(),      name="signal_watchdog"),
        asyncio.create_task(bridge.run(),               name="mavlink_bridge"),
        asyncio.create_task(collector.run(),            name="telemetry_collector"),
        asyncio.create_task(cellular.run(),             name="cellular_monitor"),
        asyncio.create_task(stream_server.run(),        name="stream_server"),
        asyncio.create_task(obstacle_detector.run(),    name="obstacle_detector"),
        asyncio.create_task(web_controller.run(),       name="web_controller"),
        asyncio.create_task(command_poller.run(),       name="command_poller"),
        asyncio.create_task(beacon.run(),               name="meshtastic_beacon"),
    ]

    # Add SITL physics loop as a background task when sim mode is active
    if sitl is not None:
        tasks.append(asyncio.create_task(sitl.run(), name="sitl_physics"))

    logger.info("River Horizon is operational. Drone '%s' ready.", config.drone_id)

    # ------------------------------------------------------------------
    # 12. Wait for shutdown signal
    # ------------------------------------------------------------------
    await _shutdown_event.wait()

    # ------------------------------------------------------------------
    # 13. Graceful shutdown
    # ------------------------------------------------------------------
    logger.info("Shutting down River Horizon...")

    # Cancel all background tasks
    for task in tasks:
        task.cancel()
    await asyncio.gather(*tasks, return_exceptions=True)

    # Teardown in reverse-init order
    await beacon.shutdown()
    await command_poller.shutdown()
    await web_controller.shutdown()
    await stream_server.shutdown()
    await camera.shutdown()
    await collector.shutdown()
    await cellular.shutdown()
    await vpn.shutdown()
    await bridge.disconnect()
    await fleet.deregister_unit(config.drone_id)
    await fault_manager.shutdown()

    logger.info("River Horizon shutdown complete. Goodbye.")


def _load_named_waypoints(waypoint_manager) -> None:
    """
    Load named waypoints from the drone profile into the waypoint manager.

    Reads the 'named_waypoints' array from drone_profile.json and registers
    each entry so they are available for River Song voice commands like
    "fly to waypoint alpha".

    Args:
        waypoint_manager: Active WaypointManager to register waypoints into.
    """
    from flight.waypoint_manager import Waypoint

    raw_waypoints = config.get("named_waypoints", [])
    if not raw_waypoints:
        logger.info("No named waypoints found in drone profile.")
        return

    loaded = 0
    for entry in raw_waypoints:
        try:
            wp = Waypoint(
                name=entry["name"],
                latitude=float(entry["latitude"]),
                longitude=float(entry["longitude"]),
                altitude_m=float(entry["altitude_m"]),
                loiter_time_s=float(entry.get("loiter_time_s", 0.0)),
            )
            waypoint_manager.register_waypoint(wp)
            loaded += 1
        except (KeyError, ValueError, TypeError) as exc:
            logger.warning("Skipping invalid waypoint entry %s: %s", entry, exc)

    logger.info("Loaded %d named waypoint(s) from drone profile.", loaded)


def main() -> None:
    """
    Synchronous entry point. Parses CLI args, validates config, then runs.

    Usage:
        python3 -m core.main          # real drone mode
        python3 -m core.main --sim    # SITL simulation mode (no drone needed)

    Raises:
        SystemExit: On configuration error or unhandled exception.
    """
    parser = argparse.ArgumentParser(
        prog="python3 -m core.main",
        description="River Horizon — Autonomous Drone Fleet Management",
    )
    parser.add_argument(
        "--sim", action="store_true", default=False,
        help="Run in SITL simulation mode (no physical drone required). "
             "Also activated by SIM_MODE=true environment variable.",
    )
    args = parser.parse_args()

    # Activate SITL if requested via flag or env var
    if args.sim or os.getenv("SIM_MODE", "").lower() in ("1", "true", "yes"):
        SITLBackend.activate()

    try:
        logger.debug("Config loaded: %r", config)
    except ConfigError as exc:
        logger.critical("Configuration error: %s", exc)
        sys.exit(1)

    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # pylint: disable=broad-except
        logger.critical("Unhandled exception in main loop: %s", exc, exc_info=True)
        sys.exit(1)


if __name__ == "__main__":
    main()
