"""
================================================================================
Project:    River Horizon — Autonomous Drone Fleet Management System
File:       safety/fault_manager.py
Purpose:    Central fault aggregator. Receives fault reports from all safety
            subsystems, logs them, forwards alerts to the alert manager, and
            dispatches the appropriate failsafe response (RTH or LAND).
            Acts as the single source of truth for the drone's fault state.
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
from typing import Callable, Coroutine, Dict, List, Optional

from core.constants import FaultReason, SystemStatus

logger = logging.getLogger(__name__)


@dataclass
class FaultRecord:
    """Immutable record of a single fault event."""
    reason: FaultReason
    message: str
    timestamp: float = field(default_factory=time.monotonic)
    resolved: bool = False
    resolved_at: Optional[float] = None


# Type alias for RTH handler coroutine
RTHHandler = Callable[[FaultReason], Coroutine]


class FaultManager:
    """
    Central fault aggregator and failsafe dispatcher.

    All safety subsystems (battery, signal, geofence) call report_fault()
    when they detect a problem. The fault manager:
    1. Records the fault in the fault log.
    2. Determines the appropriate response (RTH vs LAND).
    3. Calls the registered RTH handler.
    4. Forwards the fault to the alert manager (if registered).

    The fault manager is intentionally simple and synchronous in its
    decision logic — safety responses must be fast and predictable.
    """

    # Faults that trigger immediate LAND instead of RTH
    _IMMEDIATE_LAND_FAULTS: frozenset = frozenset({
        FaultReason.CRITICAL_BATTERY,
        FaultReason.HARDWARE_FAULT,
    })

    def __init__(self, drone_id: str) -> None:
        """
        Initialise the fault manager.

        Args:
            drone_id: Drone identifier for logging.
        """
        self._drone_id = drone_id
        self._fault_log: List[FaultRecord] = []
        self._active_faults: Dict[FaultReason, FaultRecord] = {}
        self._rth_handler: Optional[RTHHandler] = None
        self._land_handler: Optional[Callable] = None
        self._alert_manager = None
        self._lock = asyncio.Lock()
        self._overall_status: SystemStatus = SystemStatus.OK

    # ------------------------------------------------------------------
    # Handler registration
    # ------------------------------------------------------------------

    def register_rth_handler(self, handler: RTHHandler) -> None:
        """
        Register the coroutine to call when an RTH-triggering fault occurs.

        Args:
            handler: Async callable accepting a FaultReason argument.
        """
        self._rth_handler = handler
        logger.debug("[%s] RTH handler registered.", self._drone_id)

    def register_land_handler(self, handler: Callable) -> None:
        """
        Register the coroutine to call when an immediate-land fault occurs.

        Args:
            handler: Async callable with no required arguments.
        """
        self._land_handler = handler
        logger.debug("[%s] LAND handler registered.", self._drone_id)

    def register_alert_manager(self, alert_manager) -> None:
        """
        Register the alert manager for upstream fault notifications.

        Args:
            alert_manager: AlertManager instance.
        """
        self._alert_manager = alert_manager

    # ------------------------------------------------------------------
    # Fault reporting
    # ------------------------------------------------------------------

    async def report_fault(self, reason: FaultReason, message: str) -> None:
        """
        Report a fault from any safety subsystem.

        If the fault is new (not already active), it is recorded, logged,
        and the appropriate failsafe response is dispatched.

        Args:
            reason:  The FaultReason enum value.
            message: Human-readable description of the fault condition.
        """
        async with self._lock:
            if reason in self._active_faults:
                # Fault already active — don't re-trigger
                logger.debug(
                    "[%s] Fault %s already active — skipping duplicate.", self._drone_id, reason
                )
                return

            record = FaultRecord(reason=reason, message=message)
            self._fault_log.append(record)
            self._active_faults[reason] = record
            self._update_overall_status()

        logger.critical(
            "[%s] FAULT REPORTED: %s — %s", self._drone_id, reason.value, message
        )

        # Forward to alert manager
        if self._alert_manager is not None:
            try:
                await self._alert_manager.send_fault_alert(reason, message)
            except Exception as exc:  # pylint: disable=broad-except
                logger.error("[%s] Alert manager error: %s", self._drone_id, exc)

        # Dispatch failsafe response
        await self._dispatch_failsafe(reason)

    async def resolve_fault(self, reason: FaultReason) -> None:
        """
        Mark an active fault as resolved.

        Args:
            reason: The FaultReason to resolve.
        """
        async with self._lock:
            if reason not in self._active_faults:
                return
            record = self._active_faults.pop(reason)
            record.resolved = True
            record.resolved_at = time.monotonic()
            self._update_overall_status()

        logger.info("[%s] Fault resolved: %s", self._drone_id, reason.value)

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    async def shutdown(self) -> None:
        """Perform any cleanup on shutdown."""
        logger.info(
            "[%s] FaultManager shutdown. Total faults recorded: %d.",
            self._drone_id, len(self._fault_log),
        )

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def overall_status(self) -> SystemStatus:
        """Aggregate system status based on active faults."""
        return self._overall_status

    @property
    def active_faults(self) -> Dict[FaultReason, FaultRecord]:
        """Dictionary of currently active (unresolved) faults."""
        return dict(self._active_faults)

    @property
    def fault_log(self) -> List[FaultRecord]:
        """Complete chronological fault history for this session."""
        return list(self._fault_log)

    @property
    def has_active_faults(self) -> bool:
        """True if any faults are currently active."""
        return len(self._active_faults) > 0

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    async def _dispatch_failsafe(self, reason: FaultReason) -> None:
        """
        Dispatch the appropriate failsafe response for the given fault.

        Args:
            reason: The fault reason to respond to.
        """
        if reason in self._IMMEDIATE_LAND_FAULTS:
            logger.critical("[%s] Dispatching IMMEDIATE LAND for fault: %s", self._drone_id, reason)
            if self._land_handler is not None:
                try:
                    await self._land_handler()
                except Exception as exc:  # pylint: disable=broad-except
                    logger.error("[%s] LAND handler error: %s", self._drone_id, exc)
            elif self._rth_handler is not None:
                # Fallback to RTH if no land handler registered
                try:
                    await self._rth_handler(reason)
                except Exception as exc:  # pylint: disable=broad-except
                    logger.error("[%s] RTH handler error (land fallback): %s", self._drone_id, exc)
        else:
            logger.critical("[%s] Dispatching RTH for fault: %s", self._drone_id, reason)
            if self._rth_handler is not None:
                try:
                    await self._rth_handler(reason)
                except Exception as exc:  # pylint: disable=broad-except
                    logger.error("[%s] RTH handler error: %s", self._drone_id, exc)
            else:
                logger.error(
                    "[%s] No RTH handler registered — fault %s cannot be responded to.",
                    self._drone_id, reason,
                )

    def _update_overall_status(self) -> None:
        """Recompute the overall system status from active faults."""
        if not self._active_faults:
            self._overall_status = SystemStatus.OK
        elif any(r in self._IMMEDIATE_LAND_FAULTS for r in self._active_faults):
            self._overall_status = SystemStatus.CRITICAL
        else:
            self._overall_status = SystemStatus.FAILSAFE
