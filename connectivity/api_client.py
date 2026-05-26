"""
================================================================================
Project:    River Horizon — Autonomous Drone Fleet Management System
File:       connectivity/api_client.py
Purpose:    River Song API client. Handles all HTTP communication with the
            River Song AI backend under the /api/horizon/ prefix. Supports
            telemetry push, alert delivery, command polling, and fleet
            registration. Implements retry with exponential backoff.
Author:     [Author Placeholder]
Version:    1.0.0
Date:       2026-05-25
Part of:    River Song AI Ecosystem (riversongai.com)
================================================================================
"""

import asyncio
import logging
import time
from typing import Any, Dict, Optional

try:
    import aiohttp
    AIOHTTP_AVAILABLE = True
except ImportError:
    AIOHTTP_AVAILABLE = False

from core.constants import (
    API_TIMEOUT_S,
    API_RETRY_ATTEMPTS,
    API_RETRY_BACKOFF_S,
    RIVER_SONG_HEADER_KEY,
    RIVER_SONG_DRONE_ID_HEADER,
    RIVER_SONG_API_PREFIX,
)

logger = logging.getLogger(__name__)


class RiverSongAPIError(Exception):
    """Raised when the River Song API returns an error response."""


class RiverSongAPIClient:
    """
    Async HTTP client for the River Song AI API.

    All requests include authentication headers and the drone ID.
    Failed requests are retried with exponential backoff up to
    API_RETRY_ATTEMPTS times before giving up.

    If the base_url is empty or aiohttp is unavailable, all methods
    are no-ops (simulation/offline mode).
    """

    def __init__(self, base_url: str, api_key: str, drone_id: str) -> None:
        """
        Initialise the API client.

        Args:
            base_url:  Base URL of the River Song API (e.g. 'https://api.riversongai.com').
            api_key:   API authentication key (from RIVER_SONG_API_KEY env var).
            drone_id:  Drone identifier sent in every request header.
        """
        self._base_url = base_url.rstrip("/") if base_url else ""
        self._api_key = api_key
        self._drone_id = drone_id
        self._session: Optional[Any] = None
        self._enabled = bool(base_url and api_key and AIOHTTP_AVAILABLE)

        if not AIOHTTP_AVAILABLE:
            logger.warning("aiohttp not installed — River Song API client disabled.")
        elif not base_url:
            logger.info("River Song API base_url not configured — running offline.")
        elif not api_key:
            logger.warning("River Song API key not set — API client disabled.")

    # ------------------------------------------------------------------
    # Session lifecycle
    # ------------------------------------------------------------------

    async def _get_session(self) -> Any:
        """
        Return the shared aiohttp ClientSession, creating it if needed.

        Returns:
            Active aiohttp.ClientSession.
        """
        if not AIOHTTP_AVAILABLE:
            return None
        if self._session is None or self._session.closed:
            timeout = aiohttp.ClientTimeout(total=API_TIMEOUT_S)
            self._session = aiohttp.ClientSession(
                headers={
                    RIVER_SONG_HEADER_KEY: self._api_key,
                    RIVER_SONG_DRONE_ID_HEADER: self._drone_id,
                    "Content-Type": "application/json",
                },
                timeout=timeout,
            )
        return self._session

    async def close(self) -> None:
        """Close the HTTP session."""
        if self._session and not self._session.closed:
            await self._session.close()

    # ------------------------------------------------------------------
    # Public API methods
    # ------------------------------------------------------------------

    async def push_telemetry(self, packet: Dict[str, Any]) -> bool:
        """
        Push a telemetry packet to the River Song API.

        Args:
            packet: Telemetry packet dictionary.

        Returns:
            True if the packet was accepted, False on failure.
        """
        return await self._post(f"{RIVER_SONG_API_PREFIX}/telemetry", packet)

    async def send_alert(self, alert: Dict[str, Any]) -> bool:
        """
        Send an alert notification to the River Song API.

        Args:
            alert: Alert dictionary (from AlertManager).

        Returns:
            True if the alert was accepted.
        """
        return await self._post(f"{RIVER_SONG_API_PREFIX}/alerts", alert)

    async def register_drone(self, profile: Dict[str, Any]) -> bool:
        """
        Register this drone unit with the River Song fleet dashboard.

        Args:
            profile: Drone profile dictionary.

        Returns:
            True if registration succeeded.
        """
        return await self._post(f"{RIVER_SONG_API_PREFIX}/fleet/register", profile)

    async def poll_commands(self) -> Optional[Dict[str, Any]]:
        """
        Poll the River Song API for pending commands (voice commands, schedules).

        Returns:
            Command dictionary if a command is pending, None otherwise.
        """
        return await self._get(f"{RIVER_SONG_API_PREFIX}/commands/{self._drone_id}")

    async def report_status(self, status: Dict[str, Any]) -> bool:
        """
        Report the drone's current operational status to River Song.

        Args:
            status: Status dictionary including flight mode, fault state, etc.

        Returns:
            True if the status was accepted.
        """
        return await self._post(f"{RIVER_SONG_API_PREFIX}/status/{self._drone_id}", status)

    # ------------------------------------------------------------------
    # Private HTTP helpers
    # ------------------------------------------------------------------

    async def _post(self, path: str, payload: Dict[str, Any]) -> bool:
        """
        Send a POST request with retry logic.

        Args:
            path:    API path (appended to base_url).
            payload: JSON-serialisable request body.

        Returns:
            True if the request succeeded (2xx response).
        """
        if not self._enabled:
            return True  # Silent no-op in offline mode

        url = f"{self._base_url}{path}"
        for attempt in range(1, API_RETRY_ATTEMPTS + 1):
            try:
                session = await self._get_session()
                async with session.post(url, json=payload) as resp:
                    if resp.status < 300:
                        return True
                    logger.warning(
                        "API POST %s returned %d (attempt %d/%d).",
                        path, resp.status, attempt, API_RETRY_ATTEMPTS,
                    )
            except Exception as exc:  # pylint: disable=broad-except
                logger.debug(
                    "API POST %s failed (attempt %d/%d): %s",
                    path, attempt, API_RETRY_ATTEMPTS, exc,
                )

            if attempt < API_RETRY_ATTEMPTS:
                backoff = API_RETRY_BACKOFF_S * (2 ** (attempt - 1))
                await asyncio.sleep(backoff)

        logger.error("API POST %s failed after %d attempts.", path, API_RETRY_ATTEMPTS)
        return False

    async def _get(self, path: str) -> Optional[Dict[str, Any]]:
        """
        Send a GET request with retry logic.

        Args:
            path: API path (appended to base_url).

        Returns:
            Parsed JSON response body, or None on failure.
        """
        if not self._enabled:
            return None

        url = f"{self._base_url}{path}"
        for attempt in range(1, API_RETRY_ATTEMPTS + 1):
            try:
                session = await self._get_session()
                async with session.get(url) as resp:
                    if resp.status == 200:
                        return await resp.json()
                    if resp.status == 204:
                        return None  # No content — no pending commands
                    logger.warning(
                        "API GET %s returned %d (attempt %d/%d).",
                        path, resp.status, attempt, API_RETRY_ATTEMPTS,
                    )
            except Exception as exc:  # pylint: disable=broad-except
                logger.debug(
                    "API GET %s failed (attempt %d/%d): %s",
                    path, attempt, API_RETRY_ATTEMPTS, exc,
                )

            if attempt < API_RETRY_ATTEMPTS:
                backoff = API_RETRY_BACKOFF_S * (2 ** (attempt - 1))
                await asyncio.sleep(backoff)

        return None
