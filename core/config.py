"""
================================================================================
Project:    River Horizon — Autonomous Drone Fleet Management System
File:       core/config.py
Purpose:    Centralised configuration loader. Reads drone_profile.json and
            environment variables, validates required fields, and exposes a
            single typed Config object consumed by every subsystem.
Author:     [Author Placeholder]
Version:    1.0.0
Date:       2026-05-25
Part of:    River Song AI Ecosystem (riversongai.com)
================================================================================
"""

import json
import logging
import os
from pathlib import Path
from typing import Any, Optional

from dotenv import load_dotenv

from core.constants import (
    BATTERY_LOW_PCT,
    BATTERY_CRITICAL_PCT,
    HEARTBEAT_INTERVAL_S,
    MAVLINK_DEFAULT_BAUD,
    DEFAULT_STREAM_PORT,
    DEFAULT_STREAM_WIDTH,
    DEFAULT_STREAM_HEIGHT,
    DEFAULT_STREAM_FPS,
    RIVER_SONG_API_PREFIX,
)

# Load .env file if present (secrets, API keys, etc.)
load_dotenv()

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Default profile path — can be overridden via DRONE_PROFILE env var
# ---------------------------------------------------------------------------
_DEFAULT_PROFILE = Path(__file__).resolve().parent.parent / "units" / "drone_profile.json"


class ConfigError(Exception):
    """Raised when the configuration is missing required fields or is invalid."""


class FlightParamsConfig:
    """Typed wrapper for flight parameter settings."""

    def __init__(self, data: dict) -> None:
        """Initialise flight parameters from profile dict."""
        self.max_altitude_m: float      = float(data.get("max_altitude_m", 120.0))
        self.max_speed_ms: float        = float(data.get("max_speed_ms", 15.0))
        self.rth_altitude_m: float      = float(data.get("rth_altitude_m", 50.0))
        self.hover_throttle: float      = float(data.get("hover_throttle", 0.5))
        self.takeoff_altitude_m: float  = float(data.get("takeoff_altitude_m", 10.0))
        self.waypoint_radius_m: float   = float(data.get("waypoint_radius_m", 2.0))
        self.loiter_time_s: float       = float(data.get("loiter_time_s", 5.0))


class SafetyLimitsConfig:
    """Typed wrapper for safety threshold settings."""

    def __init__(self, data: dict) -> None:
        """Initialise safety limits from profile dict."""
        self.low_battery_pct: float         = float(data.get("low_battery_threshold", BATTERY_LOW_PCT))
        self.critical_battery_pct: float    = float(data.get("critical_battery_threshold", BATTERY_CRITICAL_PCT))
        self.geofence_radius_m: float       = float(data.get("geofence_radius_m", 500.0))
        self.geofence_max_altitude_m: float = float(data.get("geofence_max_altitude_m", 120.0))
        self.signal_timeout_s: float        = float(data.get("signal_timeout_s", 5.0))
        self.min_rssi_dbm: int              = int(data.get("min_rssi_dbm", -90))
        self.obstacle_stop_m: float         = float(data.get("obstacle_stop_distance_m", 3.0))


class ConnectivityConfig:
    """Typed wrapper for connectivity settings."""

    def __init__(self, data: dict) -> None:
        """Initialise connectivity settings from profile dict."""
        self.mavlink_connection_string: str = data.get("mavlink_connection_string", "/dev/ttyAMA0")
        self.baud_rate: int                 = int(data.get("baud_rate", MAVLINK_DEFAULT_BAUD))
        self.cellular_interface: str        = data.get("cellular_interface", "wwan0")
        self.vpn_enabled: bool              = bool(data.get("vpn_enabled", True))
        self.vpn_interface: str             = data.get("vpn_interface", "wg0")
        self.vpn_config_path: str           = data.get("vpn_config_path", "/etc/wireguard/wg0.conf")
        self.telemetry_host: str            = data.get("telemetry_host", "")
        self.telemetry_port: int            = int(data.get("telemetry_port", 14550))


class StreamingConfig:
    """Typed wrapper for video streaming settings."""

    def __init__(self, data: dict) -> None:
        """Initialise streaming settings from profile dict."""
        self.enabled: bool          = bool(data.get("enabled", True))
        self.port: int              = int(data.get("port", DEFAULT_STREAM_PORT))
        self.width: int             = int(data.get("width", DEFAULT_STREAM_WIDTH))
        self.height: int            = int(data.get("height", DEFAULT_STREAM_HEIGHT))
        self.fps: int               = int(data.get("fps", DEFAULT_STREAM_FPS))
        self.camera_index: int      = int(data.get("camera_index", 0))
        self.on_demand: bool        = bool(data.get("on_demand", True))


class RiverSongConfig:
    """Typed wrapper for River Song API integration settings."""

    def __init__(self, data: dict) -> None:
        """Initialise River Song API settings, preferring env vars for secrets."""
        self.base_url: str      = os.getenv("RIVER_SONG_API_URL", data.get("base_url", ""))
        self.api_key: str       = os.getenv("RIVER_SONG_API_KEY", data.get("api_key", ""))
        self.api_prefix: str    = data.get("api_prefix", RIVER_SONG_API_PREFIX)
        self.enabled: bool      = bool(data.get("enabled", True))

        if self.enabled and not self.base_url:
            logger.warning("River Song API is enabled but no base_url is configured.")
        if self.enabled and not self.api_key:
            logger.warning("River Song API is enabled but no api_key is set. "
                           "Set RIVER_SONG_API_KEY environment variable.")


class Config:
    """
    Master configuration object for a single drone unit.

    Loads from drone_profile.json and environment variables. All subsystems
    should import the module-level `config` singleton rather than instantiating
    this class directly.
    """

    def __init__(self, profile_path: Optional[Path] = None) -> None:
        """
        Load and validate the drone configuration profile.

        Args:
            profile_path: Path to drone_profile.json. Defaults to
                          units/drone_profile.json relative to project root.

        Raises:
            ConfigError: If the profile is missing or lacks required fields.
        """
        self._profile_path = Path(profile_path or os.getenv("DRONE_PROFILE", str(_DEFAULT_PROFILE)))
        self._raw: dict = self._load_profile()
        self._validate()

        self.drone_id: str          = self._raw["drone_id"]
        self.model: str             = self._raw.get("model", "Unknown")
        self.weight_kg: float       = float(self._raw.get("weight_kg", 0.0))
        self.description: str       = self._raw.get("description", "")

        self.flight     = FlightParamsConfig(self._raw.get("flight_params", {}))
        self.safety     = SafetyLimitsConfig(self._raw.get("safety_limits", {}))
        self.connectivity = ConnectivityConfig(self._raw.get("connectivity", {}))
        self.streaming  = StreamingConfig(self._raw.get("streaming", {}))
        self.river_song = RiverSongConfig(self._raw.get("river_song", {}))

        logger.info("Configuration loaded for drone '%s' (%s)", self.drone_id, self.model)

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _load_profile(self) -> dict:
        """
        Read and parse the JSON drone profile.

        Returns:
            Parsed profile as a dictionary.

        Raises:
            ConfigError: If the file is missing or contains invalid JSON.
        """
        if not self._profile_path.exists():
            raise ConfigError(
                f"Drone profile not found: {self._profile_path}. "
                "Create units/drone_profile.json before starting River Horizon."
            )
        try:
            with self._profile_path.open("r", encoding="utf-8") as fh:
                data = json.load(fh)
            logger.debug("Loaded drone profile from %s", self._profile_path)
            return data
        except json.JSONDecodeError as exc:
            raise ConfigError(f"Invalid JSON in drone profile: {exc}") from exc

    def _validate(self) -> None:
        """
        Assert that all required top-level fields are present.

        Raises:
            ConfigError: If any required field is absent.
        """
        required = ["drone_id"]
        missing = [k for k in required if k not in self._raw]
        if missing:
            raise ConfigError(f"Drone profile missing required fields: {missing}")

    # ------------------------------------------------------------------
    # Public helpers
    # ------------------------------------------------------------------

    def get(self, key: str, default: Any = None) -> Any:
        """
        Retrieve a raw top-level value from the profile.

        Args:
            key:     Top-level key in the profile JSON.
            default: Value returned if key is absent.

        Returns:
            The value associated with key, or default.
        """
        return self._raw.get(key, default)

    def reload(self) -> None:
        """
        Hot-reload the drone profile from disk without restarting.

        Useful when the profile is updated at runtime (e.g., new geofence
        coordinates pushed from River Song dashboard).
        """
        logger.info("Reloading drone profile from %s", self._profile_path)
        self._raw = self._load_profile()
        self._validate()
        self.flight     = FlightParamsConfig(self._raw.get("flight_params", {}))
        self.safety     = SafetyLimitsConfig(self._raw.get("safety_limits", {}))
        self.connectivity = ConnectivityConfig(self._raw.get("connectivity", {}))
        self.streaming  = StreamingConfig(self._raw.get("streaming", {}))
        self.river_song = RiverSongConfig(self._raw.get("river_song", {}))
        logger.info("Profile reloaded successfully for drone '%s'", self.drone_id)

    def __repr__(self) -> str:
        """Return a human-readable summary of the loaded config."""
        return (
            f"<Config drone_id={self.drone_id!r} model={self.model!r} "
            f"profile={self._profile_path}>"
        )


# ---------------------------------------------------------------------------
# Module-level singleton — import this everywhere
# ---------------------------------------------------------------------------
config = Config()
