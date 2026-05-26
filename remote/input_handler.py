"""
================================================================================
Project:    River Horizon — Autonomous Drone Fleet Management System
File:       remote/input_handler.py
Purpose:    Remote input command parser and validator. Translates raw JSON
            commands received from the web controller or River Song API into
            validated, typed command objects that the flight controller can
            execute safely.
Author:     [Author Placeholder]
Version:    1.0.0
Date:       2026-05-25
Part of:    River Song AI Ecosystem (riversongai.com)
================================================================================
"""

import logging
from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, Optional

logger = logging.getLogger(__name__)


class CommandType(str, Enum):
    """Enumeration of all supported remote command types."""
    ARM             = "arm"
    DISARM          = "disarm"
    TAKEOFF         = "takeoff"
    LAND            = "land"
    RTH             = "rth"
    GOTO            = "goto"
    GOTO_WAYPOINT   = "goto_waypoint"
    SET_VELOCITY    = "set_velocity"
    START_MISSION   = "start_mission"
    PAUSE_MISSION   = "pause_mission"
    RESUME_MISSION  = "resume_mission"
    ABORT_MISSION   = "abort_mission"
    START_STREAM    = "start_stream"
    STOP_STREAM     = "stop_stream"
    SET_MODE        = "set_mode"
    RELOAD_CONFIG   = "reload_config"


class CommandValidationError(Exception):
    """Raised when a command payload fails validation."""


@dataclass
class RemoteCommand:
    """A validated, typed remote command ready for execution."""
    command_type: CommandType
    params: Dict[str, Any]
    source: str = "web"     # "web" | "river_song" | "api"
    session_id: Optional[str] = None


class InputHandler:
    """
    Parses and validates raw JSON command payloads from remote sources.

    All commands pass through validate_and_parse() before being forwarded
    to the flight controller. Invalid or malformed commands are rejected
    with a descriptive error rather than silently ignored.
    """

    # Required parameters for each command type
    _REQUIRED_PARAMS: Dict[CommandType, list] = {
        CommandType.TAKEOFF:        ["altitude_m"],
        CommandType.GOTO:           ["latitude", "longitude", "altitude_m"],
        CommandType.GOTO_WAYPOINT:  ["name"],
        CommandType.SET_VELOCITY:   ["vx", "vy", "vz"],
        CommandType.START_MISSION:  ["mission_name"],
        CommandType.START_STREAM:   ["session_id"],
        CommandType.STOP_STREAM:    ["session_id"],
        CommandType.SET_MODE:       ["mode"],
    }

    # Parameter type constraints: {param_name: (type, min, max)}
    _PARAM_CONSTRAINTS: Dict[str, tuple] = {
        "altitude_m":   (float, 0.5, 120.0),
        "latitude":     (float, -90.0, 90.0),
        "longitude":    (float, -180.0, 180.0),
        "vx":           (float, -20.0, 20.0),
        "vy":           (float, -20.0, 20.0),
        "vz":           (float, -10.0, 10.0),
    }

    def validate_and_parse(
        self,
        raw: Dict[str, Any],
        source: str = "web",
    ) -> RemoteCommand:
        """
        Validate and parse a raw command dictionary.

        Args:
            raw:    Raw JSON-decoded command dictionary. Must contain a
                    'command' key with a valid CommandType value.
            source: Origin of the command ('web', 'river_song', 'api').

        Returns:
            A validated RemoteCommand ready for execution.

        Raises:
            CommandValidationError: If the command is missing, unknown,
                                    or has invalid parameters.
        """
        if not isinstance(raw, dict):
            raise CommandValidationError("Command payload must be a JSON object.")

        # Extract command type
        cmd_str = raw.get("command", "").lower().strip()
        if not cmd_str:
            raise CommandValidationError("Command payload missing 'command' field.")

        try:
            cmd_type = CommandType(cmd_str)
        except ValueError:
            raise CommandValidationError(
                f"Unknown command '{cmd_str}'. "
                f"Valid commands: {[c.value for c in CommandType]}"
            )

        params = raw.get("params", {})
        if not isinstance(params, dict):
            raise CommandValidationError("'params' must be a JSON object.")

        # Check required parameters
        required = self._REQUIRED_PARAMS.get(cmd_type, [])
        missing = [p for p in required if p not in params]
        if missing:
            raise CommandValidationError(
                f"Command '{cmd_type}' missing required params: {missing}"
            )

        # Validate parameter types and ranges
        validated_params = {}
        for key, value in params.items():
            validated_params[key] = self._validate_param(key, value)

        session_id = raw.get("session_id")

        logger.debug("Parsed command: %s from %s (params=%s)", cmd_type, source, validated_params)
        return RemoteCommand(
            command_type=cmd_type,
            params=validated_params,
            source=source,
            session_id=session_id,
        )

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _validate_param(self, key: str, value: Any) -> Any:
        """
        Validate and coerce a single parameter value.

        Args:
            key:   Parameter name.
            value: Raw parameter value from the command payload.

        Returns:
            Validated and coerced parameter value.

        Raises:
            CommandValidationError: If the value is out of range or wrong type.
        """
        constraint = self._PARAM_CONSTRAINTS.get(key)
        if constraint is None:
            return value  # No constraint defined — pass through

        expected_type, min_val, max_val = constraint
        try:
            coerced = expected_type(value)
        except (TypeError, ValueError):
            raise CommandValidationError(
                f"Parameter '{key}' must be {expected_type.__name__}, got {type(value).__name__}."
            )

        if not (min_val <= coerced <= max_val):
            raise CommandValidationError(
                f"Parameter '{key}' value {coerced} is out of range "
                f"[{min_val}, {max_val}]."
            )

        return coerced
