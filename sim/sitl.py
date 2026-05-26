"""
================================================================================
Project:    River Horizon — Autonomous Drone Fleet Management System
File:       sim/sitl.py
Purpose:    Software-In-The-Loop (SITL) simulation backend. Provides in-process
            drone physics and sensor simulation so the full River Horizon stack
            can run on any machine without a physical drone, flight controller,
            or cameras attached. Activated via SIM_MODE=true environment variable.
Author:     [Author Placeholder]
Version:    1.0.0
Date:       2026-05-25
Part of:    River Song AI Ecosystem (riversongai.com)
================================================================================

What it simulates
-----------------
- Vehicle state (position, altitude, heading, speed, battery, GPS)
- Flight physics: takeoff climb, level flight at target speed, landing descent
- ArduPilot-style mode transitions (GUIDED, AUTO, RTL, LAND, LOITER)
- Battery drain at 0.05 %/s in flight, 0.01 %/s idle
- Geofence-aware RTL trigger (the sim does NOT enforce geofence — the real
  GeofenceMonitor does that against SITLVehicle.position)
- Cellular RSSI (synthetic sine-wave variation around -65 dBm)
- Camera frames (colour gradient or blank numpy frames)

How to use
----------
Set SIM_MODE=true in the environment, or pass --sim on the CLI.
MAVLinkBridge, CameraManager, and CellularMonitor all check
SITLBackend.active and use the sim path automatically.

    SIM_MODE=true python3 -m core.main

Or programmatically:

    from sim.sitl import SITLBackend
    SITLBackend.activate()
"""

import asyncio
import logging
import math
import time
from dataclasses import dataclass, field
from typing import Dict, Optional

import numpy as np

logger = logging.getLogger(__name__)

# -------------------------------------------------------------------
# Sim constants
# -------------------------------------------------------------------
_CLIMB_RATE_MS      = 2.0    # m/s upward during takeoff
_DESCENT_RATE_MS    = 1.5    # m/s downward during landing
_CRUISE_ACCEL       = 0.5    # m/s² acceleration toward target speed
_BATTERY_DRAIN_FLY  = 0.05   # %/s in flight
_BATTERY_DRAIN_IDLE = 0.01   # %/s on ground / idle
_RSSI_BASE_DBM      = -65    # dBm centre for cellular signal sim
_RSSI_AMPLITUDE     = 10     # dBm variation amplitude


# -------------------------------------------------------------------
# Vehicle state model
# -------------------------------------------------------------------

@dataclass
class SITLVehicle:
    """In-memory drone state for the SITL simulation."""
    # Position
    latitude:  float = 40.712980
    longitude: float = -74.006180
    altitude_m: float = 0.0
    relative_altitude_m: float = 0.0
    heading_deg: float = 0.0
    speed_ms: float = 0.0

    # Target (used by flight physics)
    target_lat:    Optional[float] = None
    target_lng:    Optional[float] = None
    target_alt_m:  float = 0.0
    target_speed_ms: float = 0.0

    # Power
    battery_pct:     float = 100.0
    battery_voltage: float = 12.6

    # State flags
    armed:       bool = False
    flight_mode: str  = "LOITER"
    gps_fix:     int  = 3
    gps_sats:    int  = 12
    system_status: str = "OK"

    # Internal bookkeeping
    _airborne:   bool = False
    _landing:    bool = False
    _last_tick:  float = field(default_factory=time.monotonic)


class SITLBackend:
    """
    Singleton SITL backend. Activated once; all subsystems check the
    `active` flag and call methods on this shared instance.
    """

    _instance: Optional["SITLBackend"] = None
    active: bool = False

    def __init__(self) -> None:
        self.vehicle = SITLVehicle()
        self._running = False
        self._task: Optional[asyncio.Task] = None
        self._heartbeat_seq = 0

    # ------------------------------------------------------------------
    # Class-level activation
    # ------------------------------------------------------------------

    @classmethod
    def activate(cls) -> "SITLBackend":
        """
        Activate SITL mode and return the singleton instance.
        Safe to call multiple times — returns the same instance.
        """
        if cls._instance is None:
            cls._instance = cls()
        cls.active = True
        logger.info("SITL backend activated — in-process drone simulation running.")
        return cls._instance

    @classmethod
    def get(cls) -> Optional["SITLBackend"]:
        """Return the singleton if active, else None."""
        return cls._instance if cls.active else None

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    async def run(self) -> None:
        """Runs the SITL physics loop until cancelled."""
        self._running = True
        logger.info("SITL physics loop started.")
        try:
            while self._running:
                await asyncio.sleep(0.1)   # 10 Hz physics tick
                self._tick(0.1)
        except asyncio.CancelledError:
            pass
        finally:
            logger.info("SITL physics loop stopped.")

    async def shutdown(self) -> None:
        """Stop the physics loop."""
        self._running = False

    # ------------------------------------------------------------------
    # Flight commands (called by MAVLinkBridge sim path)
    # ------------------------------------------------------------------

    async def arm(self) -> bool:
        if not self.vehicle.armed:
            self.vehicle.armed = True
            self.vehicle.flight_mode = "GUIDED"
            logger.info("[SITL] Armed.")
        return True

    async def disarm(self) -> bool:
        if self.vehicle.armed and not self.vehicle._airborne:
            self.vehicle.armed = False
            self.vehicle.flight_mode = "LOITER"
            logger.info("[SITL] Disarmed.")
            return True
        if self.vehicle._airborne:
            logger.warning("[SITL] Cannot disarm while airborne.")
            return False
        return True

    async def takeoff(self, altitude_m: float) -> bool:
        if not self.vehicle.armed:
            logger.warning("[SITL] Takeoff rejected — not armed.")
            return False
        self.vehicle.target_alt_m = altitude_m
        self.vehicle._airborne = True
        self.vehicle._landing = False
        self.vehicle.flight_mode = "GUIDED"
        logger.info("[SITL] Takeoff to %.1fm.", altitude_m)
        return True

    async def land(self) -> bool:
        self.vehicle._landing = True
        self.vehicle.target_alt_m = 0.0
        self.vehicle.flight_mode = "LAND"
        logger.info("[SITL] Landing initiated.")
        return True

    async def set_mode(self, mode: str) -> bool:
        self.vehicle.flight_mode = mode
        if mode == "RTL":
            # Fly back to home (just climb to safe alt and descend)
            self.vehicle.target_alt_m = max(self.vehicle.relative_altitude_m, 30.0)
            self.vehicle._landing = False
        logger.info("[SITL] Mode → %s.", mode)
        return True

    async def goto_position(
        self,
        lat: float, lng: float, alt_m: float,
        speed_ms: Optional[float] = None,
    ) -> bool:
        self.vehicle.target_lat = lat
        self.vehicle.target_lng = lng
        self.vehicle.target_alt_m = alt_m
        self.vehicle.target_speed_ms = speed_ms or 8.0
        self.vehicle.flight_mode = "GUIDED"
        logger.info("[SITL] Goto (%.5f, %.5f) alt=%.1fm.", lat, lng, alt_m)
        return True

    # ------------------------------------------------------------------
    # Telemetry read (used by MAVLinkBridge.vehicle_state.snapshot)
    # ------------------------------------------------------------------

    def snapshot(self) -> Dict:
        v = self.vehicle
        return {
            "latitude":           v.latitude,
            "longitude":          v.longitude,
            "altitude_m":         v.altitude_m,
            "relative_altitude_m": v.relative_altitude_m,
            "heading_deg":        v.heading_deg,
            "speed_ms":           v.speed_ms,
            "battery_pct":        v.battery_pct,
            "battery_voltage":    v.battery_voltage,
            "flight_mode":        v.flight_mode,
            "armed":              v.armed,
            "gps_fix":            v.gps_fix,
            "gps_sats":           v.gps_sats,
            "system_status":      v.system_status,
            "last_heartbeat":     time.monotonic(),
        }

    # ------------------------------------------------------------------
    # RSSI simulation (called by CellularMonitor sim path)
    # ------------------------------------------------------------------

    def rssi_dbm(self) -> int:
        """Returns a slowly varying RSSI value around -65 dBm."""
        wave = math.sin(time.monotonic() / 30.0) * _RSSI_AMPLITUDE
        return int(_RSSI_BASE_DBM + wave)

    # ------------------------------------------------------------------
    # Camera frame simulation (called by CameraManager sim path)
    # ------------------------------------------------------------------

    def camera_frame(self, width: int = 1280, height: int = 720) -> np.ndarray:
        """
        Returns a synthetic camera frame — a colour gradient with
        a text overlay showing sim state.
        """
        frame = np.zeros((height, width, 3), dtype=np.uint8)

        # Sky gradient (blue → lighter blue top to bottom)
        for row in range(height // 2):
            intensity = int(180 + row * 75 / (height // 2))
            frame[row, :] = (intensity, 100, 30)   # BGR

        # Ground gradient (green top edge, brown bottom)
        for row in range(height // 2, height):
            t = (row - height // 2) / (height // 2)
            b = int(30 * (1 - t))
            g = int(100 * (1 - t) + 40 * t)
            r = int(30 * (1 - t) + 80 * t)
            frame[row, :] = (b, g, r)

        # Sim data overlay
        try:
            import cv2
            v = self.vehicle
            lines = [
                f"[SITL] {v.flight_mode}  armed={v.armed}",
                f"alt={v.relative_altitude_m:.1f}m  spd={v.speed_ms:.1f}m/s",
                f"bat={v.battery_pct:.0f}%  hdg={v.heading_deg:.0f}deg",
                f"pos=({v.latitude:.5f}, {v.longitude:.5f})",
            ]
            for i, line in enumerate(lines):
                cv2.putText(
                    frame, line, (10, 30 + i * 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2,
                )
        except ImportError:
            pass   # cv2 not available — return plain gradient frame

        return frame

    # ------------------------------------------------------------------
    # Physics tick (called every 0.1 s)
    # ------------------------------------------------------------------

    def _tick(self, dt: float) -> None:
        v = self.vehicle

        # Battery drain
        if v.armed:
            drain = _BATTERY_DRAIN_FLY if v._airborne else _BATTERY_DRAIN_IDLE
            v.battery_pct = max(0.0, v.battery_pct - drain * dt)
            v.battery_voltage = 10.0 + (v.battery_pct / 100.0) * 2.6

        if not v._airborne:
            return

        # Vertical motion
        if v._landing:
            drop = _DESCENT_RATE_MS * dt
            v.relative_altitude_m = max(0.0, v.relative_altitude_m - drop)
            if v.relative_altitude_m <= 0.05:
                v.relative_altitude_m = 0.0
                v._airborne = False
                v._landing = False
                v.armed = False
                v.flight_mode = "LOITER"
                v.speed_ms = 0.0
                logger.info("[SITL] Landed and disarmed.")
        elif v.relative_altitude_m < v.target_alt_m - 0.1:
            v.relative_altitude_m = min(
                v.target_alt_m,
                v.relative_altitude_m + _CLIMB_RATE_MS * dt,
            )
        elif v.relative_altitude_m > v.target_alt_m + 0.1:
            v.relative_altitude_m = max(
                v.target_alt_m,
                v.relative_altitude_m - _DESCENT_RATE_MS * dt,
            )

        v.altitude_m = v.relative_altitude_m  # simplified (ignores terrain)

        # Horizontal motion toward target
        if v.target_lat is not None and v.target_lng is not None:
            dlat = v.target_lat - v.latitude
            dlng = v.target_lng - v.longitude
            dist_deg = math.hypot(dlat, dlng)

            if dist_deg > 1e-7:
                # Accelerate up to target speed
                v.speed_ms = min(
                    v.target_speed_ms,
                    v.speed_ms + _CRUISE_ACCEL * dt,
                )
                # metres/degree at this latitude
                m_per_deg_lat = 111_320.0
                m_per_deg_lng = 111_320.0 * math.cos(math.radians(v.latitude))

                dist_m = math.hypot(dlat * m_per_deg_lat, dlng * m_per_deg_lng)
                step_m = v.speed_ms * dt

                if step_m >= dist_m:
                    v.latitude  = v.target_lat
                    v.longitude = v.target_lng
                    v.speed_ms  = 0.0
                else:
                    frac = step_m / dist_m
                    v.latitude  += dlat * frac
                    v.longitude += dlng * frac

                # Update heading
                v.heading_deg = (
                    math.degrees(math.atan2(dlng * m_per_deg_lng, dlat * m_per_deg_lat))
                ) % 360
            else:
                v.speed_ms = 0.0
        else:
            v.speed_ms = max(0.0, v.speed_ms - _CRUISE_ACCEL * dt)


# -------------------------------------------------------------------
# Module-level convenience: activate from env var
# -------------------------------------------------------------------
import os as _os
if _os.getenv("SIM_MODE", "").lower() in ("1", "true", "yes"):
    SITLBackend.activate()
