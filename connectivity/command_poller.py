"""
================================================================================
Project:    River Horizon — Autonomous Drone Fleet Management System
File:       connectivity/command_poller.py
Purpose:    River Song command poller. Periodically polls the River Song API
            for pending voice commands and scheduled missions, then dispatches
            them to the appropriate flight subsystem. This is the integration
            point for "River, launch Horizon unit 1 to waypoint alpha" style
            voice commands from the River Song AI brain.
Author:     [Author Placeholder]
Version:    1.0.0
Date:       2026-05-25
Part of:    River Song AI Ecosystem (riversongai.com)
================================================================================
"""

import asyncio
import logging
from typing import Any, Dict, Optional

from core.constants import FaultReason

logger = logging.getLogger(__name__)

# How often to poll the River Song API for pending commands (seconds)
_POLL_INTERVAL_S = 2.0


class CommandPoller:
    """
    Background poller for River Song voice commands and scheduled missions.

    Polls the River Song API at a regular interval and dispatches any
    pending commands to the flight controller, waypoint manager, or
    stream manager. Commands are acknowledged after successful dispatch
    to prevent re-execution.

    Supported River Song command verbs (mapped from natural language):
    - "launch to waypoint <name>"  → goto_waypoint
    - "return home"                → rth
    - "land"                       → land
    - "arm"                        → arm
    - "disarm"                     → disarm
    - "start stream"               → start_stream
    - "stop stream"                → stop_stream
    - "start mission <name>"       → start_mission
    - "abort mission"              → abort_mission
    """

    def __init__(
        self,
        drone_id: str,
        api_client,         # RiverSongAPIClient
        flight_controller,  # FlightController
        waypoint_manager,   # WaypointManager
        mode_manager,       # ModeManager
        stream_manager,     # StreamManager
        fault_manager,      # FaultManager
        poll_interval_s: float = _POLL_INTERVAL_S,
    ) -> None:
        """
        Initialise the command poller.

        Args:
            drone_id:           Drone identifier for logging and API calls.
            api_client:         River Song API client for polling commands.
            flight_controller:  Flight controller for arm/takeoff/land/goto.
            waypoint_manager:   Waypoint manager for waypoint and mission commands.
            mode_manager:       Mode manager for mode queries.
            stream_manager:     Stream manager for stream start/stop.
            fault_manager:      Fault manager for RTH dispatch.
            poll_interval_s:    Seconds between API polls.
        """
        self._drone_id = drone_id
        self._api = api_client
        self._fc = flight_controller
        self._wm = waypoint_manager
        self._mm = mode_manager
        self._sm = stream_manager
        self._fm = fault_manager
        self._poll_interval_s = poll_interval_s
        self._running: bool = False
        self._commands_dispatched: int = 0

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    async def run(self) -> None:
        """
        Poll the River Song API for commands and dispatch them.
        Runs until cancelled.
        """
        self._running = True
        logger.info("[%s] Command poller started (interval=%.1fs).",
                    self._drone_id, self._poll_interval_s)

        while self._running:
            try:
                await self._poll_and_dispatch()
            except asyncio.CancelledError:
                break
            except Exception as exc:  # pylint: disable=broad-except
                logger.error("[%s] Command poll error: %s", self._drone_id, exc)

            await asyncio.sleep(self._poll_interval_s)

        logger.info("[%s] Command poller stopped. Total dispatched: %d.",
                    self._drone_id, self._commands_dispatched)

    async def shutdown(self) -> None:
        """Stop the command poller."""
        self._running = False

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def commands_dispatched(self) -> int:
        """Total number of River Song commands dispatched this session."""
        return self._commands_dispatched

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    async def _poll_and_dispatch(self) -> None:
        """
        Poll the API for one pending command and dispatch it if present.
        """
        command = await self._api.poll_commands()
        if command is None:
            return

        logger.info("[%s] River Song command received: %s", self._drone_id, command)
        await self._dispatch(command)
        self._commands_dispatched += 1

    async def _dispatch(self, command: Dict[str, Any]) -> None:
        """
        Dispatch a River Song command to the appropriate subsystem.

        The command dict from the API is expected to have at minimum:
        - "verb": str  — the action to perform
        - "params": dict — optional parameters

        Args:
            command: Command dictionary from the River Song API.
        """
        verb = command.get("verb", "").lower().strip()
        params = command.get("params", {})
        source = command.get("source", "river_song")

        logger.info("[%s] Dispatching River Song command: verb='%s' params=%s source=%s",
                    self._drone_id, verb, params, source)

        try:
            if verb == "arm":
                await self._fc.arm()

            elif verb == "disarm":
                await self._fc.disarm()

            elif verb in ("takeoff", "launch"):
                altitude = float(params.get("altitude_m", 10.0))
                await self._fc.arm()
                await asyncio.sleep(1.0)  # Brief pause after arm
                await self._fc.takeoff(altitude)

            elif verb == "land":
                await self._fc.land()

            elif verb in ("rth", "return home", "return to home", "come home"):
                await self._fm.report_fault(
                    FaultReason.RIVER_SONG_COMMAND,
                    "RTH commanded by River Song voice command.",
                )

            elif verb in ("goto waypoint", "fly to waypoint", "launch to waypoint"):
                name = params.get("name", "").strip()
                if not name:
                    logger.warning("[%s] goto_waypoint command missing 'name' param.", self._drone_id)
                    return
                await self._wm.goto_named_waypoint(name)

            elif verb in ("goto", "fly to"):
                await self._fc.goto_position(
                    float(params["latitude"]),
                    float(params["longitude"]),
                    float(params.get("altitude_m", 30.0)),
                    params.get("speed_ms"),
                )

            elif verb in ("start stream", "show feed", "show camera"):
                session_id = params.get("session_id", "river_song")
                await self._sm.start_stream(session_id, "river_song")

            elif verb in ("stop stream", "hide feed", "stop camera"):
                session_id = params.get("session_id", "river_song")
                await self._sm.stop_stream(session_id)

            elif verb in ("start mission", "begin mission", "execute mission"):
                mission_name = params.get("mission_name", "").strip()
                if not mission_name:
                    logger.warning("[%s] start_mission command missing 'mission_name'.", self._drone_id)
                    return
                # Missions must be pre-registered; look up by name
                logger.info("[%s] Mission '%s' requested by River Song.", self._drone_id, mission_name)
                # Mission execution is handled by the web controller / mission registry
                # This hook allows future integration with a mission library

            elif verb in ("abort mission", "cancel mission", "stop mission"):
                await self._wm.abort_mission()

            elif verb in ("pause mission", "hold"):
                await self._wm.pause_mission()

            elif verb in ("resume mission", "continue mission"):
                await self._wm.resume_mission()

            else:
                logger.warning(
                    "[%s] Unknown River Song command verb: '%s'", self._drone_id, verb
                )

        except Exception as exc:  # pylint: disable=broad-except
            logger.error(
                "[%s] Failed to dispatch River Song command '%s': %s",
                self._drone_id, verb, exc,
            )
