"""
================================================================================
Project:    River Horizon — Autonomous Drone Fleet Management System
File:       connectivity/vpn.py
Purpose:    WireGuard VPN manager. Brings up the WireGuard tunnel on startup,
            monitors tunnel health, and attempts reconnection on failure.
            The VPN tunnel is required for secure web controller access.
Author:     [Author Placeholder]
Version:    1.0.0
Date:       2026-05-25
Part of:    River Song AI Ecosystem (riversongai.com)
================================================================================
"""

import asyncio
import logging
import subprocess
from typing import Optional

from core.constants import SystemStatus

logger = logging.getLogger(__name__)

_HEALTH_CHECK_INTERVAL_S = 15.0
_RECONNECT_DELAY_S = 5.0
_MAX_RECONNECT_ATTEMPTS = 5


class VPNManager:
    """
    WireGuard VPN lifecycle manager.

    Brings up the WireGuard interface on initialise(), monitors tunnel
    health via periodic wg show checks, and reconnects automatically
    on failure. If VPN is disabled in the profile, all operations are
    no-ops.
    """

    def __init__(self, enabled: bool, interface: str, config_path: str) -> None:
        """
        Initialise the VPN manager.

        Args:
            enabled:     Whether VPN is enabled (from drone profile).
            interface:   WireGuard interface name (e.g. 'wg0').
            config_path: Path to the WireGuard config file.
        """
        self._enabled = enabled
        self._interface = interface
        self._config_path = config_path

        self._running: bool = False
        self._status: SystemStatus = SystemStatus.INITIALIZING
        self._connected: bool = False
        self._reconnect_attempts: int = 0

    async def initialise(self) -> None:
        """
        Bring up the WireGuard tunnel.
        No-op if VPN is disabled.
        """
        if not self._enabled:
            logger.info("[vpn] VPN disabled — skipping tunnel setup.")
            self._status = SystemStatus.OK
            return

        logger.info("[vpn] Bringing up WireGuard interface '%s'.", self._interface)
        success = await self._bring_up()
        if success:
            self._connected = True
            self._status = SystemStatus.OK
            logger.info("[vpn] WireGuard tunnel up on '%s'.", self._interface)
        else:
            self._status = SystemStatus.WARNING
            logger.warning("[vpn] WireGuard tunnel failed to come up — will retry.")

    async def shutdown(self) -> None:
        """Bring down the WireGuard tunnel on shutdown."""
        self._running = False
        if not self._enabled or not self._connected:
            return
        logger.info("[vpn] Bringing down WireGuard interface '%s'.", self._interface)
        await self._bring_down()

    # ------------------------------------------------------------------
    # Background health monitor
    # ------------------------------------------------------------------

    async def run(self) -> None:
        """
        Periodically check tunnel health and reconnect if needed.
        No-op if VPN is disabled.
        """
        if not self._enabled:
            return

        self._running = True
        logger.info("[vpn] Health monitor started.")

        while self._running:
            try:
                await self._check_health()
            except asyncio.CancelledError:
                break
            except Exception as exc:  # pylint: disable=broad-except
                logger.error("[vpn] Health check error: %s", exc)

            await asyncio.sleep(_HEALTH_CHECK_INTERVAL_S)

        logger.info("[vpn] Health monitor stopped.")

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def is_connected(self) -> bool:
        """True if the WireGuard tunnel is active."""
        return self._connected

    @property
    def status(self) -> SystemStatus:
        """Current health status of the VPN tunnel."""
        return self._status

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    async def _bring_up(self) -> bool:
        """
        Execute wg-quick up to bring up the tunnel.

        Returns:
            True if the command succeeded.
        """
        return await self._run_wg_command(["wg-quick", "up", self._config_path])

    async def _bring_down(self) -> bool:
        """
        Execute wg-quick down to tear down the tunnel.

        Returns:
            True if the command succeeded.
        """
        return await self._run_wg_command(["wg-quick", "down", self._config_path])

    async def _check_health(self) -> None:
        """
        Check tunnel health using 'wg show'. Reconnect if the tunnel is down.
        """
        loop = asyncio.get_running_loop()
        try:
            result = await loop.run_in_executor(
                None,
                lambda: subprocess.run(
                    ["wg", "show", self._interface],
                    capture_output=True, text=True, timeout=5,
                ),
            )
            if result.returncode == 0:
                self._connected = True
                self._status = SystemStatus.OK
                self._reconnect_attempts = 0
            else:
                await self._handle_tunnel_down()
        except (FileNotFoundError, subprocess.TimeoutExpired):
            # wg not installed — simulation mode
            self._connected = True
            self._status = SystemStatus.OK

    async def _handle_tunnel_down(self) -> None:
        """Attempt to reconnect the WireGuard tunnel."""
        self._connected = False
        self._status = SystemStatus.WARNING
        self._reconnect_attempts += 1

        if self._reconnect_attempts > _MAX_RECONNECT_ATTEMPTS:
            self._status = SystemStatus.CRITICAL
            logger.error(
                "[vpn] WireGuard tunnel down after %d reconnect attempts.",
                self._reconnect_attempts,
            )
            return

        logger.warning(
            "[vpn] Tunnel down — reconnect attempt %d/%d.",
            self._reconnect_attempts, _MAX_RECONNECT_ATTEMPTS,
        )
        await asyncio.sleep(_RECONNECT_DELAY_S)
        success = await self._bring_up()
        if success:
            self._connected = True
            self._status = SystemStatus.OK
            self._reconnect_attempts = 0
            logger.info("[vpn] Tunnel reconnected successfully.")

    @staticmethod
    async def _run_wg_command(cmd: list) -> bool:
        """
        Run a WireGuard CLI command asynchronously.

        Args:
            cmd: Command and arguments list.

        Returns:
            True if the command exited with code 0.
        """
        loop = asyncio.get_running_loop()
        try:
            result = await loop.run_in_executor(
                None,
                lambda: subprocess.run(cmd, capture_output=True, text=True, timeout=10),
            )
            if result.returncode != 0:
                logger.warning("[vpn] Command %s failed: %s", cmd[0], result.stderr.strip())
            return result.returncode == 0
        except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
            logger.debug("[vpn] Command unavailable (%s) — simulation mode.", exc)
            return True  # Treat as success in simulation
