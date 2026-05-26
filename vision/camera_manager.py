"""
================================================================================
Project:    River Horizon — Autonomous Drone Fleet Management System
File:       vision/camera_manager.py
Purpose:    Camera hardware abstraction layer. Manages OpenCV VideoCapture,
            exposes async frame reads, and handles camera open/close lifecycle.
            Supports both physical cameras (CSI, USB) and RTSP sources.
Author:     [Author Placeholder]
Version:    1.0.0
Date:       2026-05-25
Part of:    River Song AI Ecosystem (riversongai.com)
================================================================================
"""

import asyncio
import logging
from typing import Optional, Tuple

import numpy as np

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False

logger = logging.getLogger(__name__)


class CameraError(Exception):
    """Raised when the camera cannot be opened or a frame cannot be read."""


class CameraManager:
    """
    Async camera manager backed by OpenCV VideoCapture.

    Provides thread-safe async frame reads via run_in_executor so the
    blocking cv2.VideoCapture.read() call does not block the event loop.
    Falls back to generating blank frames if OpenCV is unavailable.
    """

    def __init__(
        self,
        camera_index: int = 0,
        width: int = 1280,
        height: int = 720,
        fps: int = 30,
    ) -> None:
        """
        Initialise the camera manager.

        Args:
            camera_index: OpenCV camera index or RTSP URL string.
            width:        Capture width in pixels.
            height:       Capture height in pixels.
            fps:          Target capture frame rate.
        """
        self._camera_index = camera_index
        self._width = width
        self._height = height
        self._fps = fps

        self._cap = None
        self._open: bool = False
        self._frame_count: int = 0
        self._lock = asyncio.Lock()

    async def initialise(self) -> None:
        """
        Open the camera device.

        Raises:
            CameraError: If the camera cannot be opened.
        """
        if not CV2_AVAILABLE:
            logger.warning("OpenCV not available — camera running in simulation mode.")
            self._open = True
            return

        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, self._open_camera)

        if not self._open:
            raise CameraError(
                f"Failed to open camera at index/source '{self._camera_index}'."
            )
        logger.info(
            "Camera opened: index=%s, %dx%d @ %dfps.",
            self._camera_index, self._width, self._height, self._fps,
        )

    async def read_frame(self) -> Optional[np.ndarray]:
        """
        Read the next frame from the camera asynchronously.

        Returns:
            BGR frame as a numpy array, or None if the read failed.
        """
        if not self._open:
            return None

        if not CV2_AVAILABLE:
            # Return a blank frame in simulation mode
            return np.zeros((self._height, self._width, 3), dtype=np.uint8)

        async with self._lock:
            loop = asyncio.get_running_loop()
            ret, frame = await loop.run_in_executor(None, self._cap.read)
            if ret:
                self._frame_count += 1
                return frame
            logger.warning("Camera read failed (frame %d).", self._frame_count)
            return None

    async def shutdown(self) -> None:
        """Release the camera device."""
        if self._cap and CV2_AVAILABLE:
            loop = asyncio.get_running_loop()
            await loop.run_in_executor(None, self._cap.release)
        self._open = False
        logger.info("Camera released.")

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def is_open(self) -> bool:
        """True if the camera is currently open."""
        return self._open

    @property
    def resolution(self) -> Tuple[int, int]:
        """Camera resolution as (width, height)."""
        return (self._width, self._height)

    @property
    def frame_count(self) -> int:
        """Total frames read since the camera was opened."""
        return self._frame_count

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _open_camera(self) -> None:
        """
        Synchronous camera open (called via executor).
        Sets self._open to True on success.
        """
        self._cap = cv2.VideoCapture(self._camera_index)
        if not self._cap.isOpened():
            self._open = False
            return

        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
        self._cap.set(cv2.CAP_PROP_FPS, self._fps)
        self._open = True
