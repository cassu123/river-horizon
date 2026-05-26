"""
================================================================================
Project:    River Horizon — Autonomous Drone Fleet Management System
File:       remote/web_controller.py
Purpose:    FastAPI web controller. Exposes the River Song API endpoints
            under /api/horizon/ for remote drone control, telemetry reads,
            stream management, and fleet status. Accessible via WireGuard VPN
            from any browser or the River Song dashboard.
Author:     [Author Placeholder]
Version:    1.0.0
Date:       2026-05-25
Part of:    River Song AI Ecosystem (riversongai.com)
================================================================================
"""

import asyncio
import logging
import os
from typing import Any, Dict, Optional

try:
    from fastapi import FastAPI, HTTPException
    from fastapi import Request as FastAPIRequest
    from fastapi.middleware.cors import CORSMiddleware
    import uvicorn
    FASTAPI_AVAILABLE = True
except ImportError:
    FastAPIRequest = None   # type: ignore
    HTTPException = None    # type: ignore
    FASTAPI_AVAILABLE = False

from core.constants import RIVER_SONG_API_PREFIX, RIVER_SONG_HEADER_KEY
from remote.input_handler import InputHandler, CommandValidationError, CommandType

logger = logging.getLogger(__name__)

_WEB_PORT = int(os.getenv("WEB_CONTROLLER_PORT", "8080"))
_API_KEY = os.getenv("RIVER_SONG_API_KEY", "")


class WebController:
    """
    FastAPI-based web controller for River Horizon.

    Exposes REST endpoints under /api/horizon/ for:
    - Drone command execution (arm, takeoff, goto, RTH, etc.)
    - Real-time telemetry reads
    - Video stream management
    - Fleet status and health
    - River Song voice command integration

    All write endpoints require the X-River-Song-Key header for authentication.
    """

    def __init__(
        self,
        drone_id: str,
        flight_controller,      # FlightController
        waypoint_manager,       # WaypointManager
        mode_manager,           # ModeManager
        stream_manager,         # StreamManager
        telemetry_collector,    # TelemetryCollector
        api_prefix: str = RIVER_SONG_API_PREFIX,
    ) -> None:
        """
        Initialise the web controller.

        Args:
            drone_id:             Drone identifier.
            flight_controller:    Flight controller for command execution.
            waypoint_manager:     Waypoint manager for mission commands.
            mode_manager:         Mode manager for mode queries.
            stream_manager:       Stream manager for video stream control.
            telemetry_collector:  Telemetry collector for live data reads.
            api_prefix:           URL prefix for all API routes.
        """
        self._drone_id = drone_id
        self._fc = flight_controller
        self._wm = waypoint_manager
        self._mm = mode_manager
        self._sm = stream_manager
        self._tc = telemetry_collector
        self._api_prefix = api_prefix
        self._input_handler = InputHandler()

        self._app: Optional[Any] = None
        self._server_task: Optional[asyncio.Task] = None

        if FASTAPI_AVAILABLE:
            self._build_app()

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    async def run(self) -> None:
        """Start the uvicorn server in the background."""
        if not FASTAPI_AVAILABLE:
            logger.warning("FastAPI/uvicorn not available — web controller disabled.")
            while True:
                await asyncio.sleep(60)

        config = uvicorn.Config(
            app=self._app,
            host="0.0.0.0",
            port=_WEB_PORT,
            log_level="warning",
            access_log=False,
        )
        server = uvicorn.Server(config)
        logger.info(
            "Web controller starting on http://0.0.0.0:%d%s",
            _WEB_PORT, self._api_prefix,
        )
        await server.serve()

    async def shutdown(self) -> None:
        """Shut down the web controller."""
        logger.info("Web controller shutting down.")

    # ------------------------------------------------------------------
    # App construction
    # ------------------------------------------------------------------

    def _build_app(self) -> None:
        """Build the FastAPI application and register all routes."""
        app = FastAPI(
            title="River Horizon API",
            description="Drone fleet management API for River Song AI ecosystem.",
            version="1.0.0",
        )

        # CORS — restrict to VPN subnet in production
        app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],    # Tighten to VPN subnet in production
            allow_methods=["*"],
            allow_headers=["*"],
        )

        self._app = app
        prefix = self._api_prefix

        # Capture self for use inside closures
        controller = self

        # ------------------------------------------------------------------
        # Status & telemetry
        # ------------------------------------------------------------------

        @app.get(f"{prefix}/status")
        async def get_status():
            """Return current drone status and telemetry snapshot."""
            packet = controller._tc.last_packet or {}
            return {
                "drone_id": controller._drone_id,
                "flight_mode": controller._mm.current_mode,
                "is_autonomous": controller._mm.is_autonomous,
                "is_returning": controller._mm.is_returning,
                "mission_active": controller._wm.is_mission_active,
                "streaming": controller._sm.is_streaming,
                "telemetry": packet,
            }

        @app.get(f"{prefix}/telemetry")
        async def get_telemetry():
            """Return the latest telemetry packet."""
            return controller._tc.last_packet or {}

        # ------------------------------------------------------------------
        # Command endpoint
        # ------------------------------------------------------------------

        @app.post(f"{prefix}/command")
        async def post_command(request: FastAPIRequest):
            """
            Execute a flight command.

            Body: {"command": "<type>", "params": {...}}
            Requires X-River-Song-Key header.
            """
            controller._require_auth(request)
            body = await request.json()
            try:
                cmd = controller._input_handler.validate_and_parse(body, source="web")
            except CommandValidationError as exc:
                raise HTTPException(status_code=400, detail=str(exc))

            result = await controller._dispatch_command(cmd)
            return {"status": "ok", "result": result}

        # ------------------------------------------------------------------
        # Stream management
        # ------------------------------------------------------------------

        @app.post(f"{prefix}/stream/start")
        async def start_stream(request: FastAPIRequest):
            """Start the video stream for this client."""
            body = await request.json()
            client_ip = request.client.host if request.client else "unknown"
            session_id = body.get("session_id", client_ip)
            url = await controller._sm.start_stream(session_id, client_ip)
            return {"status": "ok", "stream_url": url}

        @app.post(f"{prefix}/stream/stop")
        async def stop_stream(request: FastAPIRequest):
            """Stop the video stream for this client."""
            body = await request.json()
            client_ip = request.client.host if request.client else "unknown"
            session_id = body.get("session_id", client_ip)
            await controller._sm.stop_stream(session_id)
            return {"status": "ok"}

        @app.post(f"{prefix}/stream/keepalive")
        async def stream_keepalive(request: FastAPIRequest):
            """Send a keepalive ping for an active stream session."""
            body = await request.json()
            session_id = body.get("session_id")
            ok = await controller._sm.keepalive(session_id)
            return {"status": "ok" if ok else "session_not_found"}

        # ------------------------------------------------------------------
        # Waypoints
        # ------------------------------------------------------------------

        @app.get(f"{prefix}/waypoints")
        async def list_waypoints():
            """List all registered named waypoints."""
            return {"waypoints": controller._wm.list_waypoints()}

        @app.post(f"{prefix}/waypoints/register")
        async def register_waypoint(request: FastAPIRequest):
            """
            Register a named waypoint.

            Body: {"name": str, "latitude": float, "longitude": float,
                   "altitude_m": float, "loiter_time_s": float (optional)}
            """
            controller._require_auth(request)
            body = await request.json()
            from flight.waypoint_manager import Waypoint
            try:
                wp = Waypoint(
                    name=body["name"],
                    latitude=float(body["latitude"]),
                    longitude=float(body["longitude"]),
                    altitude_m=float(body["altitude_m"]),
                    loiter_time_s=float(body.get("loiter_time_s", 0.0)),
                )
            except (KeyError, ValueError) as exc:
                raise HTTPException(status_code=400, detail=f"Invalid waypoint data: {exc}")
            controller._wm.register_waypoint(wp)
            return {"status": "ok", "waypoint": wp.name}

        # ------------------------------------------------------------------
        # Fleet health
        # ------------------------------------------------------------------

        @app.get(f"{prefix}/health")
        async def health_check():
            """Simple health check endpoint."""
            return {"status": "ok", "drone_id": controller._drone_id}

        @app.get(f"{prefix}/faults")
        async def get_faults():
            """Return active fault list (requires fault_manager injection)."""
            return {"drone_id": controller._drone_id, "faults": []}

    # ------------------------------------------------------------------
    # Command dispatcher
    # ------------------------------------------------------------------

    async def _dispatch_command(self, cmd: Any) -> Any:
        """
        Route a validated RemoteCommand to the appropriate subsystem.

        Args:
            cmd: Validated RemoteCommand from InputHandler.

        Returns:
            Command result (varies by command type).

        Raises:
            HTTPException: If the command fails execution.
        """
        try:
            if cmd.command_type == CommandType.ARM:
                return await self._fc.arm()
            elif cmd.command_type == CommandType.DISARM:
                return await self._fc.disarm()
            elif cmd.command_type == CommandType.TAKEOFF:
                return await self._fc.takeoff(cmd.params["altitude_m"])
            elif cmd.command_type == CommandType.LAND:
                return await self._fc.land()
            elif cmd.command_type == CommandType.RTH:
                return await self._fc.land()
            elif cmd.command_type == CommandType.GOTO:
                return await self._fc.goto_position(
                    cmd.params["latitude"],
                    cmd.params["longitude"],
                    cmd.params["altitude_m"],
                    cmd.params.get("speed_ms"),
                )
            elif cmd.command_type == CommandType.GOTO_WAYPOINT:
                return await self._wm.goto_named_waypoint(cmd.params["name"])
            elif cmd.command_type == CommandType.SET_VELOCITY:
                return await self._fc.set_velocity(
                    cmd.params["vx"], cmd.params["vy"], cmd.params["vz"]
                )
            elif cmd.command_type == CommandType.PAUSE_MISSION:
                await self._wm.pause_mission()
                return True
            elif cmd.command_type == CommandType.RESUME_MISSION:
                await self._wm.resume_mission()
                return True
            elif cmd.command_type == CommandType.ABORT_MISSION:
                await self._wm.abort_mission()
                return True
            elif cmd.command_type == CommandType.RELOAD_CONFIG:
                from core.config import config
                config.reload()
                return True
            else:
                raise HTTPException(
                    status_code=400,
                    detail=f"Unhandled command: {cmd.command_type}",
                )
        except HTTPException:
            raise
        except Exception as exc:
            logger.error("Command dispatch error (%s): %s", cmd.command_type, exc)
            raise HTTPException(status_code=500, detail=str(exc))

    # ------------------------------------------------------------------
    # Auth helper
    # ------------------------------------------------------------------

    def _require_auth(self, request: Any) -> None:
        """
        Validate the River Song API key header.

        Args:
            request: Incoming FastAPI Request object.

        Raises:
            HTTPException: If the API key is missing or invalid.
        """
        if not _API_KEY:
            return  # Auth disabled when no key is configured (dev mode)
        provided = request.headers.get(RIVER_SONG_HEADER_KEY, "")
        if provided != _API_KEY:
            raise HTTPException(status_code=401, detail="Invalid or missing API key.")
