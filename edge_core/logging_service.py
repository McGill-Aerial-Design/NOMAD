"""
Mission Event Logging Service for NOMAD.

Handles evidence logging for competition tasks.
Saves JSON files with timestamped filenames for mission events.

Target: Python 3.13 | NVIDIA Jetson Orin Nano
"""

from __future__ import annotations

import glob
import json
import os
import re
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any

# Default log directory (relative to project root)
DEFAULT_LOG_DIR = Path(__file__).parent.parent / "data" / "mission_logs"

# Log rotation settings
MAX_LOG_DIR_SIZE_MB = 500
MAX_LOG_AGE_HOURS = 48


def strip_non_ascii(text: str) -> str:
    """
    Remove all non-ASCII characters (including emojis) from text.
    
    Args:
        text: Input string potentially containing emojis/non-ASCII
        
    Returns:
        ASCII-only string
    """
    return re.sub(r'[^\x00-\x7F]+', '', text)


def get_timestamp_filename(prefix: str = "event", extension: str = "json") -> str:
    """
    Generate a timestamped filename.

    Args:
        prefix: Filename prefix (e.g., "task1", "capture")
        extension: File extension without dot

    Returns:
        Filename like "task1_20251207_143052_123456.json"
    """
    now = datetime.now(timezone.utc)
    timestamp = now.strftime("%Y%m%d_%H%M%S_%f")
    return f"{prefix}_{timestamp}.{extension}"


def ensure_log_directory(log_dir: Path | str | None = None) -> Path:
    """
    Ensure the log directory exists.

    Args:
        log_dir: Custom log directory path. Uses default if None.

    Returns:
        Path to the log directory
    """
    path = Path(log_dir) if log_dir else DEFAULT_LOG_DIR
    path.mkdir(parents=True, exist_ok=True)
    return path


def log_mission_event(
    task_id: str | int,
    data: dict[str, Any],
    log_dir: Path | str | None = None,
) -> Path:
    """
    Log a mission event to a JSON file.

    Creates a timestamped JSON file containing the event data
    in the mission logs directory. Strips non-ASCII characters from all text.

    Args:
        task_id: Task identifier (e.g., "1", "2", "recon")
        data: Event data dictionary to log
        log_dir: Custom log directory. Uses default if None.

    Returns:
        Path to the created log file

    Example:
        >>> path = log_mission_event("1", {
        ...     "target_text": "Target is 1.5m North of Red Car",
        ...     "gps": {"lat": 45.5, "lon": -73.5}
        ... })
        >>> print(path)
        data/mission_logs/task1_20251207_143052_123456.json
    """
    # Ensure directory exists
    log_path = ensure_log_directory(log_dir)

    # Generate filename
    filename = get_timestamp_filename(prefix=f"task{task_id}")
    file_path = log_path / filename

    # Recursively strip non-ASCII from data
    cleaned_data = _clean_dict_recursive(data)

    # Prepare event data with metadata
    event = {
        "task_id": str(task_id),
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "data": cleaned_data,
    }

    # Write JSON file
    with file_path.open("w", encoding="utf-8") as f:
        json.dump(event, f, indent=2, default=str)

    return file_path


def _clean_dict_recursive(obj: Any) -> Any:
    """
    Recursively clean non-ASCII characters from dictionaries, lists, and strings.
    
    Args:
        obj: Object to clean (dict, list, str, or other)
        
    Returns:
        Cleaned object with non-ASCII chars removed from all strings
    """
    if isinstance(obj, dict):
        return {k: _clean_dict_recursive(v) for k, v in obj.items()}
    elif isinstance(obj, list):
        return [_clean_dict_recursive(item) for item in obj]
    elif isinstance(obj, str):
        return strip_non_ascii(obj)
    else:
        return obj


def log_task1_capture(
    target_text: str,
    target_gps: dict[str, float],
    drone_state: dict[str, Any],
    log_dir: Path | str | None = None,
) -> Path:
    """
    Log a Task 1 capture event with full details.

    Args:
        target_text: Generated relative text description
        target_gps: Target GPS coordinates {lat, lon, alt}
        drone_state: Drone state at capture time
        log_dir: Custom log directory

    Returns:
        Path to created log file
    """
    data = {
        "target_text": target_text,
        "target_gps": target_gps,
        "drone_state": drone_state,
    }

    return log_mission_event(task_id="1", data=data, log_dir=log_dir)


def list_mission_logs(
    task_id: str | int | None = None,
    log_dir: Path | str | None = None,
) -> list[Path]:
    """
    List all mission log files, optionally filtered by task.

    Args:
        task_id: Filter by task ID. None returns all logs.
        log_dir: Custom log directory

    Returns:
        List of log file paths sorted by timestamp (newest first)
    """
    log_path = ensure_log_directory(log_dir)

    if task_id is not None:
        pattern = f"task{task_id}_*.json"
    else:
        pattern = "task*.json"

    files = list(log_path.glob(pattern))
    return sorted(files, reverse=True)


def read_mission_log(file_path: Path | str) -> dict[str, Any]:
    """
    Read a mission log file.

    Args:
        file_path: Path to the log file

    Returns:
        Parsed JSON data
    """
    path = Path(file_path)
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def cleanup_old_logs(log_dir: str | None = None) -> int:
    """
    Remove logs older than MAX_LOG_AGE_HOURS or if directory exceeds MAX_LOG_DIR_SIZE_MB.
    
    This function safely cleans up old log files to prevent disk filling.
    Errors during cleanup do not raise exceptions (fail-safe for startup).
    
    Args:
        log_dir: Custom log directory. Default: ~/nomad_logs
        
    Returns:
        Number of files deleted
        
    Example:
        >>> deleted = cleanup_old_logs()
        >>> print(f"Deleted {deleted} old log files")
    """
    if log_dir is None:
        log_dir = str(DEFAULT_LOG_DIR)
    
    deleted = 0
    
    try:
        # Ensure directory exists
        if not os.path.isdir(log_dir):
            return 0
        
        cutoff = datetime.now() - timedelta(hours=MAX_LOG_AGE_HOURS)

        # Find all log files (*.json mission logs)
        log_files = glob.glob(os.path.join(log_dir, "*.json"))

        # Phase 1: age-based deletion.
        survivors: list[tuple[str, float, int]] = []  # (path, mtime, size)
        for log_file in log_files:
            try:
                mtime = os.path.getmtime(log_file)
                if datetime.fromtimestamp(mtime) < cutoff:
                    size = os.path.getsize(log_file)
                    os.remove(log_file)
                    deleted += 1
                    continue
                size = os.path.getsize(log_file)
                survivors.append((log_file, mtime, size))
            except OSError:
                # Skip files we can't access (don't crash)
                pass

        # Phase 2: directory-size cap. Delete oldest survivors until total
        # bytes drop below MAX_LOG_DIR_SIZE_MB.
        size_budget = MAX_LOG_DIR_SIZE_MB * 1024 * 1024
        total_size = sum(s for _, _, s in survivors)
        if total_size > size_budget:
            # Oldest first so newest mission evidence is preserved longest.
            survivors.sort(key=lambda t: t[1])
            for path, _mtime, size in survivors:
                if total_size <= size_budget:
                    break
                try:
                    os.remove(path)
                    deleted += 1
                    total_size -= size
                except OSError:
                    pass
    except Exception:
        # Fail silently - cleanup errors should not stop startup
        pass

    return deleted
