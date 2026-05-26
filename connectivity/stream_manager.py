"""
================================================================================
Project:    River Horizon — Autonomous Drone Fleet Management System
File:       connectivity/stream_manager.py
Purpose:    Video stream session manager. Tracks active stream consumers,
            manages on-demand stream lifecycle (start/stop), and provides
            stream URLs to the web controller and River Song dashboard.
            Streams are off by default to conserve 4G LTE bandwidth.
Author:     [Author Placeholder]
Version:    1.0.0
Date:       2026-05-25
Part of:    River Song AI Ecosystem (riversongai.com)
================================================================================
"""

import asyncio
import logging
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set

logger = logging.getLogger(__name__)


@dataclass
class StreamSession:
    """Represents a single active video stream consumer session."""
    session_id: str
    drone_id: str
    consumer_ip: str
    started_at: float = field(default_factory=time.time)
    last_keepalive: float = field(default_factory=time.time)

    @property
    def duration_s(self) -> float:
        """Elapsed seconds since the stream session started."""
        return time.time() - self.started_at

    @property
    def is_stale(self) -> bool:
        """True if no keepalive has been received in the last 30 seconds."""
        return time.time() - self.last_keepalive > 30.0


class StreamManager:
    """
    On-demand video stream session manager.

    Tracks which consumers are actively viewing a drone's video feed.
    When the last consumer disconnects, the stream server is signalled
    to stop encoding (saving CPU and bandwidth on the Pi).

    Supports multiple simultaneous consumers per drone (e.g. River Song
    dashboard + operator browser tab).
    """

    def __init__(self, drone_id: str) -> None:
        """
        Initialise the stream manager.

        Args:
            drone_id: Drone identifier for this manager instance.
        """
        self._drone_id = drone_id
        self._sessions: Dict[str, StreamSession] = {}
        self._stream_active: bool = False
        self._stream_url: Optional[str] = None
        self._lock = asyncio.Lock()

        # Callbacks registered by StreamServer
        self._on_start_callback = None
        self._on_stop_callback = None

    # ------------------------------------------------------------------
    # Session management
    # ------------------------------------------------------------------

    async def start_stream(self, session_id: str, consumer_ip: str) -> Optional[str]:
        """
        Register a new stream consumer and start the stream if not already running.

        Args:
            session_id:   Unique identifier for this consumer session.
            consumer_ip:  IP address of the requesting consumer.

        Returns:
            The stream URL, or None if the stream could not be started.
        """
        async with self._lock:
            if session_id in self._sessions:
                logger.debug("[%s] Stream session '%s' already active.", self._drone_id, session_id)
                return self._stream_url

            session = StreamSession(
                session_id=session_id,
                drone_id=self._drone_id,
                consumer_ip=consumer_ip,
            )
            self._sessions[session_id] = session

            if not self._stream_active:
                await self._activate_stream()

            logger.info(
                "[%s] Stream session started: %s from %s (total consumers: %d).",
                self._drone_id, session_id, consumer_ip, len(self._sessions),
            )
            return self._stream_url

    async def stop_stream(self, session_id: str) -> None:
        """
        Unregister a stream consumer. Stops the stream if no consumers remain.

        Args:
            session_id: The session ID to remove.
        """
        async with self._lock:
            if session_id not in self._sessions:
                return

            del self._sessions[session_id]
            logger.info(
                "[%s] Stream session ended: %s (remaining consumers: %d).",
                self._drone_id, session_id, len(self._sessions),
            )

            if not self._sessions and self._stream_active:
                await self._deactivate_stream()

    async def keepalive(self, session_id: str) -> bool:
        """
        Update the keepalive timestamp for a stream session.

        Args:
            session_id: The session to keep alive.

        Returns:
            True if the session exists and was updated.
        """
        async with self._lock:
            if session_id not in self._sessions:
                return False
            self._sessions[session_id].last_keepalive = time.time()
            return True

    async def prune_stale_sessions(self) -> None:
        """Remove stream sessions that have not sent a keepalive recently."""
        async with self._lock:
            stale = [sid for sid, s in self._sessions.items() if s.is_stale]
            for sid in stale:
                del self._sessions[sid]
                logger.info("[%s] Pruned stale stream session: %s", self._drone_id, sid)

            if stale and not self._sessions and self._stream_active:
                await self._deactivate_stream()

    # ------------------------------------------------------------------
    # Callback registration
    # ------------------------------------------------------------------

    def register_start_callback(self, callback) -> None:
        """
        Register a callback to invoke when the stream should start.

        Args:
            callback: Async callable with no arguments.
        """
        self._on_start_callback = callback

    def register_stop_callback(self, callback) -> None:
        """
        Register a callback to invoke when the stream should stop.

        Args:
            callback: Async callable with no arguments.
        """
        self._on_stop_callback = callback

    def set_stream_url(self, url: str) -> None:
        """
        Set the stream URL returned to consumers.

        Args:
            url: The RTSP or HTTP stream URL.
        """
        self._stream_url = url

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def is_streaming(self) -> bool:
        """True if the video stream is currently active."""
        return self._stream_active

    @property
    def active_session_count(self) -> int:
        """Number of currently active stream consumer sessions."""
        return len(self._sessions)

    @property
    def stream_url(self) -> Optional[str]:
        """The current stream URL, or None if not streaming."""
        return self._stream_url if self._stream_active else None

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    async def _activate_stream(self) -> None:
        """Signal the stream server to start encoding."""
        self._stream_active = True
        logger.info("[%s] Activating video stream.", self._drone_id)
        if self._on_start_callback:
            try:
                await self._on_start_callback()
            except Exception as exc:  # pylint: disable=broad-except
                logger.error("[%s] Stream start callback error: %s", self._drone_id, exc)

    async def _deactivate_stream(self) -> None:
        """Signal the stream server to stop encoding."""
        self._stream_active = False
        logger.info("[%s] Deactivating video stream (no consumers).", self._drone_id)
        if self._on_stop_callback:
            try:
                await self._on_stop_callback()
            except Exception as exc:  # pylint: disable=broad-except
                logger.error("[%s] Stream stop callback error: %s", self._drone_id, exc)
