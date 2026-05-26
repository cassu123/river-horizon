"""
================================================================================
Project:    River Horizon — Autonomous Drone Fleet Management System
File:       vision/stream_server.py
Purpose:    MJPEG-over-HTTP stream server. Serves live video frames from the
            CameraManager to web browser clients. Integrates with StreamManager
            for on-demand activation. Accessible via WireGuard VPN from any
            browser without additional plugins.
Author:     [Author Placeholder]
Version:    1.0.0
Date:       2026-05-25
Part of:    River Song AI Ecosystem (riversongai.com)
================================================================================
"""

import asyncio
import logging
import time
from typing import Any, Optional, Set

try:
    from aiohttp import web as _aiohttp_web
    AIOHTTP_AVAILABLE = True
except ImportError:
    _aiohttp_web = None  # type: ignore
    AIOHTTP_AVAILABLE = False

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False

from core.constants import STREAM_JPEG_QUALITY, STREAM_KEEPALIVE_INTERVAL_S
from vision.camera_manager import CameraManager

logger = logging.getLogger(__name__)

_MJPEG_BOUNDARY = b"--frame"
_MJPEG_CONTENT_TYPE = "multipart/x-mixed-replace; boundary=frame"


class StreamServer:
    """
    MJPEG-over-HTTP live video stream server.

    Serves a multipart MJPEG stream at /stream that any browser can display
    using a standard <img> tag. The stream is only active when at least one
    consumer is registered with the StreamManager (on-demand mode).

    Access URL: http://<vpn-ip>:<port>/stream
    """

    def __init__(
        self,
        camera_manager: CameraManager,
        port: int,
        on_demand: bool,
        stream_manager,  # StreamManager
    ) -> None:
        """
        Initialise the stream server.

        Args:
            camera_manager: Active CameraManager for frame reads.
            port:           HTTP port to listen on.
            on_demand:      If True, only encode when consumers are connected.
            stream_manager: StreamManager for session tracking.
        """
        self._camera = camera_manager
        self._port = port
        self._on_demand = on_demand
        self._stream_manager = stream_manager

        self._encoding: bool = False
        self._latest_jpeg: Optional[bytes] = None
        self._frame_lock = asyncio.Lock()
        self._active_clients: Set[str] = set()

        self._app: Optional[Any] = None
        self._runner: Optional[Any] = None
        self._site: Optional[Any] = None

        # Register start/stop callbacks with stream manager
        stream_manager.register_start_callback(self._start_encoding)
        stream_manager.register_stop_callback(self._stop_encoding)

        # Set stream URL
        stream_manager.set_stream_url(f"http://0.0.0.0:{port}/stream")

    async def initialise(self) -> None:
        """Set up the aiohttp web application routes."""
        if not AIOHTTP_AVAILABLE:
            logger.warning("aiohttp not available — stream server disabled.")
            return

        self._app = _aiohttp_web.Application()
        self._app.router.add_get("/stream", self._handle_stream)
        self._app.router.add_get("/snapshot", self._handle_snapshot)
        self._app.router.add_get("/health", self._handle_health)
        logger.info("Stream server initialised on port %d.", self._port)

    async def run(self) -> None:
        """
        Start the HTTP server and the frame capture loop.
        Runs until shutdown() is called.
        """
        if not AIOHTTP_AVAILABLE or self._app is None:
            # Keep the task alive without doing anything
            while True:
                await asyncio.sleep(60)

        self._runner = _aiohttp_web.AppRunner(self._app)
        await self._runner.setup()
        self._site = _aiohttp_web.TCPSite(self._runner, "0.0.0.0", self._port)
        await self._site.start()
        logger.info("Stream server listening on http://0.0.0.0:%d", self._port)

        # If not on-demand, start encoding immediately
        if not self._on_demand:
            await self._start_encoding()

        # Frame capture loop
        while True:
            if self._encoding:
                await self._capture_frame()
            else:
                await asyncio.sleep(0.1)

    async def shutdown(self) -> None:
        """Stop the HTTP server and release resources."""
        self._encoding = False
        if self._runner:
            await self._runner.cleanup()
        logger.info("Stream server stopped.")

    # ------------------------------------------------------------------
    # HTTP handlers
    # ------------------------------------------------------------------

    async def _handle_stream(self, request: Any) -> Any:
        """
        Handle MJPEG stream requests.

        Args:
            request: Incoming HTTP request.

        Returns:
            Streaming multipart response.
        """
        response = _aiohttp_web.StreamResponse(
            headers={"Content-Type": _MJPEG_CONTENT_TYPE}
        )
        await response.prepare(request)

        client_id = f"{request.remote}_{time.time()}"
        self._active_clients.add(client_id)
        logger.info("Stream client connected: %s (total: %d)", request.remote, len(self._active_clients))

        try:
            while True:
                async with self._frame_lock:
                    jpeg = self._latest_jpeg

                if jpeg:
                    await response.write(
                        _MJPEG_BOUNDARY + b"\r\n"
                        b"Content-Type: image/jpeg\r\n"
                        b"Content-Length: " + str(len(jpeg)).encode() + b"\r\n\r\n"
                        + jpeg + b"\r\n"
                    )
                await asyncio.sleep(1.0 / 30)  # ~30 fps ceiling
        except (ConnectionResetError, asyncio.CancelledError):
            pass
        finally:
            self._active_clients.discard(client_id)
            logger.info("Stream client disconnected: %s (remaining: %d)",
                        request.remote, len(self._active_clients))

        return response

    async def _handle_snapshot(self, request: Any) -> Any:
        """
        Return a single JPEG snapshot.

        Args:
            request: Incoming HTTP request.

        Returns:
            JPEG image response.
        """
        async with self._frame_lock:
            jpeg = self._latest_jpeg

        if jpeg is None:
            return _aiohttp_web.Response(status=503, text="No frame available")
        return _aiohttp_web.Response(body=jpeg, content_type="image/jpeg")

    async def _handle_health(self, request: Any) -> Any:
        """
        Health check endpoint.

        Args:
            request: Incoming HTTP request.

        Returns:
            JSON health status response.
        """
        return _aiohttp_web.json_response({
            "status": "ok",
            "encoding": self._encoding,
            "active_clients": len(self._active_clients),
        })

    # ------------------------------------------------------------------
    # Encoding control
    # ------------------------------------------------------------------

    async def _start_encoding(self) -> None:
        """Start the frame capture loop."""
        self._encoding = True
        logger.info("Stream encoding started.")

    async def _stop_encoding(self) -> None:
        """Stop the frame capture loop."""
        self._encoding = False
        async with self._frame_lock:
            self._latest_jpeg = None
        logger.info("Stream encoding stopped.")

    async def _capture_frame(self) -> None:
        """Read a frame from the camera and encode it as JPEG."""
        frame = await self._camera.read_frame()
        if frame is None:
            await asyncio.sleep(0.033)
            return

        if CV2_AVAILABLE:
            loop = asyncio.get_running_loop()
            jpeg = await loop.run_in_executor(
                None,
                lambda: cv2.imencode(
                    ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, STREAM_JPEG_QUALITY]
                )[1].tobytes(),
            )
        else:
            jpeg = b""  # Simulation

        async with self._frame_lock:
            self._latest_jpeg = jpeg

        await asyncio.sleep(1.0 / 30)
