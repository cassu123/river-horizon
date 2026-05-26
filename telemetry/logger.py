"""
================================================================================
Project:    River Horizon — Autonomous Drone Fleet Management System
File:       telemetry/logger.py
Purpose:    Structured telemetry logger. Writes telemetry packets to rotating
            JSONL (newline-delimited JSON) log files on disk. Each drone gets
            its own log file. Supports async writes to avoid blocking the
            telemetry collection loop.
Author:     [Author Placeholder]
Version:    1.0.0
Date:       2026-05-25
Part of:    River Song AI Ecosystem (riversongai.com)
================================================================================
"""

import asyncio
import json
import logging
import time
from pathlib import Path
from typing import Any, Dict, Optional

logger = logging.getLogger(__name__)

# Maximum log file size before rotation (bytes)
_MAX_LOG_SIZE = 50 * 1024 * 1024   # 50 MB
_MAX_LOG_FILES = 10


class TelemetryLogger:
    """
    Async structured telemetry logger.

    Writes telemetry packets as newline-delimited JSON (JSONL) to rotating
    log files. Each session creates a new log file named with the drone ID
    and session start timestamp.

    File format: one JSON object per line, each representing one telemetry
    packet. This format is directly ingestible by tools like jq, pandas,
    and most log aggregation platforms.
    """

    def __init__(self, drone_id: str, log_dir: Path) -> None:
        """
        Initialise the telemetry logger.

        Args:
            drone_id: Drone identifier used in log file names.
            log_dir:  Directory where telemetry log files are written.
        """
        self._drone_id = drone_id
        self._log_dir = Path(log_dir)
        self._log_dir.mkdir(parents=True, exist_ok=True)

        self._current_file: Optional[Path] = None
        self._file_handle = None
        self._write_lock = asyncio.Lock()
        self._bytes_written: int = 0
        self._packets_written: int = 0
        self._session_start = time.time()

        self._open_new_file()

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    async def log(self, packet: Dict[str, Any]) -> None:
        """
        Write a telemetry packet to the current log file.

        Rotates the log file if the size limit has been reached.

        Args:
            packet: Telemetry packet dictionary to serialise and write.
        """
        async with self._write_lock:
            if self._should_rotate():
                self._rotate()

            try:
                line = json.dumps(packet, default=str) + "\n"
                encoded = line.encode("utf-8")
                loop = asyncio.get_running_loop()
                await loop.run_in_executor(None, self._file_handle.write, encoded)
                self._bytes_written += len(encoded)
                self._packets_written += 1
            except Exception as exc:  # pylint: disable=broad-except
                logger.error("[%s] Failed to write telemetry packet: %s", self._drone_id, exc)

    async def flush(self) -> None:
        """Flush the current log file buffer to disk."""
        async with self._write_lock:
            if self._file_handle:
                loop = asyncio.get_running_loop()
                await loop.run_in_executor(None, self._file_handle.flush)

    def close(self) -> None:
        """Close the current log file handle."""
        if self._file_handle:
            try:
                self._file_handle.close()
            except Exception as exc:  # pylint: disable=broad-except
                logger.warning("[%s] Error closing telemetry log: %s", self._drone_id, exc)
            self._file_handle = None

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def current_log_path(self) -> Optional[Path]:
        """Path to the currently active log file."""
        return self._current_file

    @property
    def packets_written(self) -> int:
        """Total telemetry packets written this session."""
        return self._packets_written

    @property
    def bytes_written(self) -> int:
        """Total bytes written to log files this session."""
        return self._bytes_written

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _open_new_file(self) -> None:
        """Open a new log file with a timestamped name."""
        if self._file_handle:
            try:
                self._file_handle.close()
            except Exception:  # pylint: disable=broad-except
                pass

        timestamp = time.strftime("%Y%m%dT%H%M%S", time.gmtime())
        filename = f"telemetry_{self._drone_id}_{timestamp}.jsonl"
        self._current_file = self._log_dir / filename
        self._file_handle = open(self._current_file, "ab")  # noqa: WPS515
        self._bytes_written = 0
        logger.info("[%s] Telemetry log opened: %s", self._drone_id, self._current_file)

    def _should_rotate(self) -> bool:
        """Return True if the current log file has exceeded the size limit."""
        return self._bytes_written >= _MAX_LOG_SIZE

    def _rotate(self) -> None:
        """Rotate to a new log file and prune old files if needed."""
        logger.info(
            "[%s] Rotating telemetry log (size=%.1f MB).",
            self._drone_id, self._bytes_written / (1024 * 1024),
        )
        self._open_new_file()
        self._prune_old_logs()

    def _prune_old_logs(self) -> None:
        """Delete the oldest log files if the count exceeds _MAX_LOG_FILES."""
        pattern = f"telemetry_{self._drone_id}_*.jsonl"
        logs = sorted(self._log_dir.glob(pattern), key=lambda p: p.stat().st_mtime)
        while len(logs) > _MAX_LOG_FILES:
            oldest = logs.pop(0)
            try:
                oldest.unlink()
                logger.info("[%s] Pruned old telemetry log: %s", self._drone_id, oldest.name)
            except Exception as exc:  # pylint: disable=broad-except
                logger.warning("[%s] Failed to prune log %s: %s", self._drone_id, oldest, exc)
