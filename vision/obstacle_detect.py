"""
================================================================================
Project:    River Horizon — Autonomous Drone Fleet Management System
File:       vision/obstacle_detect.py
Purpose:    Computer vision obstacle detection using OpenCV. Detects moving
            or stationary obstacles in the camera frame using background
            subtraction and contour analysis. Emits obstacle events that
            the flight controller can act on.
Author:     [Author Placeholder]
Version:    1.0.0
Date:       2026-05-25
Part of:    River Song AI Ecosystem (riversongai.com)
================================================================================
"""

import asyncio
import logging
from dataclasses import dataclass, field
from typing import Callable, List, Optional

import numpy as np

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False

from core.constants import (
    OBSTACLE_MIN_AREA_PX,
    OBSTACLE_STOP_DISTANCE_M,
    OBSTACLE_WARN_DISTANCE_M,
)
from vision.camera_manager import CameraManager

logger = logging.getLogger(__name__)


@dataclass
class ObstacleEvent:
    """Represents a detected obstacle in the camera frame."""
    area_px: float
    center_x: int
    center_y: int
    estimated_distance_m: Optional[float] = None
    severity: str = "WARNING"   # "WARNING" | "CRITICAL"


class ObstacleDetector:
    """
    Frame-by-frame obstacle detector using OpenCV background subtraction.

    Uses MOG2 background subtractor to detect moving objects and contour
    analysis to filter noise. Emits ObstacleEvent objects to registered
    callbacks when obstacles are detected.

    Note: Distance estimation is approximate (based on contour area) and
    should be supplemented with a depth sensor for production use.
    """

    def __init__(self, camera_manager: CameraManager) -> None:
        """
        Initialise the obstacle detector.

        Args:
            camera_manager: Active CameraManager for frame reads.
        """
        self._camera = camera_manager
        self._running: bool = False
        self._callbacks: List[Callable[[ObstacleEvent], None]] = []
        self._obstacle_active: bool = False

        if CV2_AVAILABLE:
            self._bg_subtractor = cv2.createBackgroundSubtractorMOG2(
                history=100, varThreshold=40, detectShadows=False
            )
        else:
            self._bg_subtractor = None

    def register_callback(self, callback: Callable[[ObstacleEvent], None]) -> None:
        """
        Register a callback to receive obstacle detection events.

        Args:
            callback: Callable that accepts an ObstacleEvent.
        """
        self._callbacks.append(callback)

    async def run(self) -> None:
        """
        Continuously process camera frames for obstacle detection.
        Runs until cancelled.
        """
        self._running = True
        logger.info("Obstacle detector started.")

        while self._running:
            try:
                frame = await self._camera.read_frame()
                if frame is not None and CV2_AVAILABLE:
                    await self._process_frame(frame)
            except asyncio.CancelledError:
                break
            except Exception as exc:  # pylint: disable=broad-except
                logger.error("Obstacle detection error: %s", exc)

            await asyncio.sleep(0.05)  # ~20 Hz processing rate

        logger.info("Obstacle detector stopped.")

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def obstacle_detected(self) -> bool:
        """True if an obstacle is currently detected in the frame."""
        return self._obstacle_active

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    async def _process_frame(self, frame: np.ndarray) -> None:
        """
        Apply background subtraction and contour detection to a frame.

        Args:
            frame: BGR frame from the camera.
        """
        loop = asyncio.get_running_loop()
        events = await loop.run_in_executor(None, self._detect_sync, frame)

        if events:
            self._obstacle_active = True
            for event in events:
                logger.warning(
                    "Obstacle detected: area=%.0fpx, center=(%d,%d), severity=%s",
                    event.area_px, event.center_x, event.center_y, event.severity,
                )
                for cb in self._callbacks:
                    try:
                        cb(event)
                    except Exception as exc:  # pylint: disable=broad-except
                        logger.error("Obstacle callback error: %s", exc)
        else:
            self._obstacle_active = False

    def _detect_sync(self, frame: np.ndarray) -> List[ObstacleEvent]:
        """
        Synchronous obstacle detection (runs in executor thread).

        Args:
            frame: BGR frame to analyse.

        Returns:
            List of ObstacleEvent objects for detected obstacles.
        """
        events: List[ObstacleEvent] = []

        # Apply background subtraction
        fg_mask = self._bg_subtractor.apply(frame)

        # Morphological cleanup to remove noise
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_OPEN, kernel)
        fg_mask = cv2.dilate(fg_mask, kernel, iterations=2)

        # Find contours
        contours, _ = cv2.findContours(fg_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < OBSTACLE_MIN_AREA_PX:
                continue

            # Compute centroid
            M = cv2.moments(contour)
            if M["m00"] == 0:
                continue
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            # Rough distance estimate: larger area = closer obstacle
            # This is a heuristic — replace with depth sensor data in production
            estimated_dist = self._estimate_distance(area)
            severity = "CRITICAL" if estimated_dist <= OBSTACLE_STOP_DISTANCE_M else "WARNING"

            events.append(ObstacleEvent(
                area_px=area,
                center_x=cx,
                center_y=cy,
                estimated_distance_m=estimated_dist,
                severity=severity,
            ))

        return events

    @staticmethod
    def _estimate_distance(area_px: float) -> float:
        """
        Estimate obstacle distance from contour area (heuristic).

        This is a rough approximation. For accurate distance measurement,
        integrate a depth sensor (e.g. TF-Luna LiDAR or stereo camera).

        Args:
            area_px: Contour area in pixels.

        Returns:
            Estimated distance in metres.
        """
        # Empirical constant — calibrate for your specific camera and lens
        _AREA_DISTANCE_CONSTANT = 500_000.0
        if area_px <= 0:
            return 999.0
        return _AREA_DISTANCE_CONSTANT / area_px
