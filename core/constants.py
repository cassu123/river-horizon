"""
================================================================================
Project:    River Horizon — Autonomous Drone Fleet Management System
File:       core/constants.py
Purpose:    System-wide constants, enumerations, and fixed configuration values.
            All magic numbers and string literals live here — never inline.
Author:     [Author Placeholder]
Version:    1.0.0
Date:       2026-05-25
Part of:    River Song AI Ecosystem (riversongai.com)
================================================================================
"""

from enum import Enum, auto


# ---------------------------------------------------------------------------
# Flight Modes
# ---------------------------------------------------------------------------

class FlightMode(str, Enum):
    """MAVLink-compatible flight mode identifiers."""
    MANUAL      = "MANUAL"
    STABILIZE   = "STABILIZE"
    ALT_HOLD    = "ALT_HOLD"
    LOITER      = "LOITER"
    GUIDED      = "GUIDED"
    AUTO        = "AUTO"
    RTL         = "RTL"       # Return to Launch
    LAND        = "LAND"
    POSHOLD     = "POSHOLD"
    BRAKE       = "BRAKE"
    ACRO        = "ACRO"


# ---------------------------------------------------------------------------
# System Status Levels
# ---------------------------------------------------------------------------

class SystemStatus(str, Enum):
    """Operational health status codes used across all subsystems."""
    OK          = "OK"
    WARNING     = "WARNING"
    CRITICAL    = "CRITICAL"
    FAILSAFE    = "FAILSAFE"
    OFFLINE     = "OFFLINE"
    INITIALIZING = "INITIALIZING"


# ---------------------------------------------------------------------------
# Fault / Trigger Reasons
# ---------------------------------------------------------------------------

class FaultReason(str, Enum):
    """Enumerated reasons that can trigger a failsafe or RTH event."""
    LOW_BATTERY         = "LOW_BATTERY"
    CRITICAL_BATTERY    = "CRITICAL_BATTERY"
    SIGNAL_LOSS         = "SIGNAL_LOSS"
    GEOFENCE_BREACH     = "GEOFENCE_BREACH"
    OBSTACLE_DETECTED   = "OBSTACLE_DETECTED"
    HARDWARE_FAULT      = "HARDWARE_FAULT"
    MANUAL_OVERRIDE     = "MANUAL_OVERRIDE"
    RIVER_SONG_COMMAND  = "RIVER_SONG_COMMAND"


# ---------------------------------------------------------------------------
# Geofence
# ---------------------------------------------------------------------------

class GeofenceAction(str, Enum):
    """Action taken when a geofence boundary is violated."""
    RTL     = "RTL"
    LOITER  = "LOITER"
    LAND    = "LAND"
    WARN    = "WARN"

class GeofenceType(str, Enum):
    """Supported geofence geometry types."""
    CYLINDER    = "CYLINDER"   # Circular radius + altitude ceiling
    POLYGON     = "POLYGON"    # Arbitrary polygon boundary


# ---------------------------------------------------------------------------
# Connectivity
# ---------------------------------------------------------------------------

class ConnectionState(str, Enum):
    """State of a network or MAVLink connection."""
    CONNECTED       = "CONNECTED"
    DISCONNECTED    = "DISCONNECTED"
    RECONNECTING    = "RECONNECTING"
    DEGRADED        = "DEGRADED"


# ---------------------------------------------------------------------------
# Telemetry
# ---------------------------------------------------------------------------

class TelemetryField(str, Enum):
    """Canonical field names used in telemetry packets."""
    DRONE_ID        = "drone_id"
    TIMESTAMP       = "timestamp"
    LATITUDE        = "latitude"
    LONGITUDE       = "longitude"
    ALTITUDE_M      = "altitude_m"
    HEADING_DEG     = "heading_deg"
    SPEED_MS        = "speed_ms"
    BATTERY_PCT     = "battery_pct"
    BATTERY_VOLTAGE = "battery_voltage"
    FLIGHT_MODE     = "flight_mode"
    SIGNAL_RSSI     = "signal_rssi"
    GPS_FIX         = "gps_fix"
    GPS_SATS        = "gps_sats"
    ARMED           = "armed"
    SYSTEM_STATUS   = "system_status"


# ---------------------------------------------------------------------------
# Timing & Rate Constants
# ---------------------------------------------------------------------------

HEARTBEAT_INTERVAL_S        = 1.0       # MAVLink heartbeat period (seconds)
TELEMETRY_RATE_HZ           = 10        # Telemetry publish rate
SAFETY_POLL_INTERVAL_S      = 0.5       # Safety subsystem polling period
SIGNAL_CHECK_INTERVAL_S     = 1.0       # Cellular signal quality check period
BATTERY_CHECK_INTERVAL_S    = 2.0       # Battery monitor polling period
GEOFENCE_CHECK_INTERVAL_S   = 0.5       # Geofence enforcement polling period
STREAM_KEEPALIVE_INTERVAL_S = 5.0       # Video stream keepalive ping period
API_TIMEOUT_S               = 10.0      # Default HTTP request timeout
API_RETRY_ATTEMPTS          = 3         # Number of retries for River Song API calls
API_RETRY_BACKOFF_S         = 2.0       # Exponential backoff base (seconds)


# ---------------------------------------------------------------------------
# MAVLink
# ---------------------------------------------------------------------------

MAVLINK_SYSTEM_ID           = 1         # GCS system ID
MAVLINK_COMPONENT_ID        = 1         # GCS component ID
MAVLINK_DEFAULT_BAUD        = 57600
MAVLINK_HEARTBEAT_TIMEOUT_S = 5.0       # Seconds before declaring link lost


# ---------------------------------------------------------------------------
# Battery Thresholds (defaults — overridden by drone_profile.json)
# ---------------------------------------------------------------------------

BATTERY_LOW_PCT             = 20.0      # Trigger RTH
BATTERY_CRITICAL_PCT        = 10.0      # Trigger immediate land
BATTERY_FULL_PCT            = 100.0


# ---------------------------------------------------------------------------
# Video Streaming
# ---------------------------------------------------------------------------

DEFAULT_STREAM_PORT         = 8554      # RTSP default port
DEFAULT_STREAM_WIDTH        = 1280
DEFAULT_STREAM_HEIGHT       = 720
DEFAULT_STREAM_FPS          = 30
STREAM_JPEG_QUALITY         = 80        # MJPEG quality (0-100)


# ---------------------------------------------------------------------------
# River Song API
# ---------------------------------------------------------------------------

RIVER_SONG_API_PREFIX       = "/api/horizon"
RIVER_SONG_HEADER_KEY       = "X-River-Song-Key"
RIVER_SONG_DRONE_ID_HEADER  = "X-Drone-ID"


# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------

LOG_FORMAT = "%(asctime)s | %(levelname)-8s | %(name)s | %(message)s"
LOG_DATE_FORMAT = "%Y-%m-%dT%H:%M:%S"
LOG_MAX_BYTES   = 10 * 1024 * 1024     # 10 MB per log file
LOG_BACKUP_COUNT = 5


# ---------------------------------------------------------------------------
# Obstacle Detection
# ---------------------------------------------------------------------------

OBSTACLE_MIN_AREA_PX        = 500       # Minimum contour area to flag as obstacle
OBSTACLE_STOP_DISTANCE_M    = 3.0       # Halt if obstacle within this range
OBSTACLE_WARN_DISTANCE_M    = 8.0       # Warn if obstacle within this range


# ---------------------------------------------------------------------------
# Miscellaneous
# ---------------------------------------------------------------------------

EARTH_RADIUS_M              = 6_371_000.0   # Mean Earth radius for haversine calc
METERS_PER_DEGREE_LAT       = 111_320.0     # Approximate at equator
