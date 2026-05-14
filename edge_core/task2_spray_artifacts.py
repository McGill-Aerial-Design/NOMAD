"""
task2_spray_artifacts.py
========================

Manages Task 2 spray-session artifacts:
  - Before image (snapshot taken at session start)
  - After image  (snapshot taken at session end)
  - Optional video recording of the session, captured via ffmpeg from
    the local RTSP stream (best-effort — silently skipped if ffmpeg
    is unavailable).

Used by two flows:
  1. Manual:    POST /api/task/2/spray/manual/{start,stop} from the
                Mission Planner Submit panel. Operator hand-flies and
                sprays; the panel uploads the resulting artifacts to
                Google Drive.
  2. Autonomous: spray_controller calls record_pre_spray() and
                record_post_spray() at the appropriate state transitions
                so artifacts are captured automatically.

Artifacts and the active session live in process memory + on disk under
~/.nomad/spray_sessions/<session_id>/.
"""

from __future__ import annotations

import logging
import os
import shutil
import subprocess
import threading
import time
import uuid
from dataclasses import dataclass, field, asdict
from typing import Callable, Optional

logger = logging.getLogger("edge_core.task2_spray_artifacts")

SESSIONS_ROOT = os.path.expanduser("~/.nomad/spray_sessions")
DEFAULT_RTSP_URL = os.environ.get(
    "NOMAD_RTSP_URL", "rtsp://172.17.0.1:8554/primary"
)


def _retention_keep_last() -> int:
    """How many of the most recent spray sessions to retain on disk."""
    raw = os.environ.get("NOMAD_SPRAY_KEEP_LAST", "20")
    try:
        n = int(raw)
        return max(1, n)
    except (TypeError, ValueError):
        return 20


def _dir_size_bytes(path: str) -> int:
    total = 0
    try:
        for root, _dirs, files in os.walk(path):
            for f in files:
                try:
                    total += os.path.getsize(os.path.join(root, f))
                except OSError:
                    pass
    except OSError:
        pass
    return total


def sweep_old_sessions(keep_last: Optional[int] = None) -> None:
    """Delete all but the most recent `keep_last` spray sessions.

    Called once at process start and again on every session finalisation so
    ~/.nomad/spray_sessions/ stays bounded across multi-day flights.
    """
    keep = keep_last if keep_last is not None else _retention_keep_last()
    try:
        if not os.path.isdir(SESSIONS_ROOT):
            return
        entries = []
        for name in os.listdir(SESSIONS_ROOT):
            full = os.path.join(SESSIONS_ROOT, name)
            if not os.path.isdir(full):
                continue
            try:
                mtime = os.path.getmtime(full)
            except OSError:
                continue
            entries.append((mtime, full))
        if len(entries) <= keep:
            return
        entries.sort(reverse=True)  # newest first
        to_delete = entries[keep:]
        freed_bytes = 0
        for _mtime, full in to_delete:
            freed_bytes += _dir_size_bytes(full)
            try:
                shutil.rmtree(full)
            except OSError as e:
                logger.warning(f"Spray retention: failed to remove {full}: {e}")
        logger.info(
            "Spray retention sweep: removed %d session(s), freed %.1f MB "
            "(kept newest %d under %s)",
            len(to_delete), freed_bytes / (1024 * 1024), keep, SESSIONS_ROOT,
        )
    except Exception as e:
        logger.warning(f"Spray retention sweep failed: {e}")


@dataclass
class SpraySession:
    session_id: str
    started_at: float
    ended_at: Optional[float] = None
    before_image_path: Optional[str] = None
    after_image_path: Optional[str] = None
    video_path: Optional[str] = None
    source: str = "manual"  # "manual" | "autonomous"

    def to_dict(self) -> dict:
        d = asdict(self)
        return d


class SprayArtifactManager:
    """Holds the current and last spray session."""

    def __init__(self, capture_photo_fn: Optional[Callable[[], Optional[str]]] = None):
        os.makedirs(SESSIONS_ROOT, exist_ok=True)
        self._capture_photo_fn = capture_photo_fn
        self._lock = threading.Lock()
        self._current: Optional[SpraySession] = None
        self._last:    Optional[SpraySession] = None
        self._ffmpeg_proc: Optional[subprocess.Popen] = None
        # Bound disk usage at startup — old runs might have accumulated.
        sweep_old_sessions()

    # ------------------------------------------------------------------ #
    def set_capture_photo_fn(self, fn: Callable[[], Optional[str]]) -> None:
        self._capture_photo_fn = fn

    # ------------------------------------------------------------------ #
    def start_session(
        self,
        source: str = "manual",
        before_path: Optional[str] = None,
    ) -> SpraySession:
        """Begin a new session.

        If `before_path` is provided (autonomous flow), the existing snapshot
        is adopted as the before-image — no second capture is taken. If it
        is None (manual flow), `_capture_photo_fn` is invoked once.

        Always starts ffmpeg video recording (best-effort).
        """
        with self._lock:
            if self._current is not None:
                # Auto-finalise stale session so a new one can take over.
                logger.warning("Starting new session while another is active; finalising stale one")
                self._finalise_locked(after_path=None)

            sid = uuid.uuid4().hex[:12]
            sess_dir = os.path.join(SESSIONS_ROOT, sid)
            os.makedirs(sess_dir, exist_ok=True)
            sess = SpraySession(
                session_id=sid,
                started_at=time.time(),
                source=source,
            )

            sess.before_image_path = self._adopt_or_capture(
                provided=before_path, sess_dir=sess_dir, name="before.jpg"
            )

            # Start ffmpeg video recording (best-effort)
            sess.video_path = self._start_recording(sess_dir)

            self._current = sess
            return sess

    def stop_session(self, after_path: Optional[str] = None) -> Optional[SpraySession]:
        """Finalise the active session.

        If `after_path` is provided (autonomous flow), the existing post-spray
        snapshot is adopted as the after-image — no second capture is taken.
        If it is None (manual flow), `_capture_photo_fn` is invoked once.
        """
        with self._lock:
            return self._finalise_locked(after_path=after_path)

    def _finalise_locked(self, after_path: Optional[str]) -> Optional[SpraySession]:
        if self._current is None:
            return None
        sess = self._current
        sess_dir = os.path.join(SESSIONS_ROOT, sess.session_id)

        sess.after_image_path = self._adopt_or_capture(
            provided=after_path, sess_dir=sess_dir, name="after.jpg"
        )

        self._stop_recording()
        # Validate that ffmpeg actually produced a non-empty file.
        if sess.video_path and (
            not os.path.exists(sess.video_path)
            or os.path.getsize(sess.video_path) < 1024
        ):
            sess.video_path = None

        sess.ended_at = time.time()
        self._last = sess
        self._current = None
        logger.info(f"Spray session {sess.session_id} finalised: {sess.to_dict()}")
        # Keep ~/.nomad/spray_sessions bounded — runs every finalisation.
        sweep_old_sessions()
        return sess

    # ------------------------------------------------------------------ #
    def current(self) -> Optional[SpraySession]:
        with self._lock:
            return self._current

    def last(self) -> Optional[SpraySession]:
        with self._lock:
            return self._last or self._current

    def _adopt_or_capture(
        self,
        provided: Optional[str],
        sess_dir: str,
        name: str,
    ) -> Optional[str]:
        """If a snapshot path was provided by the spray controller, copy it
        into the session directory; otherwise capture a fresh snapshot."""
        try:
            src = provided
            if src is None and self._capture_photo_fn is not None:
                src = self._capture_photo_fn()
            if not src or not os.path.exists(src):
                return None
            target = os.path.join(sess_dir, name)
            # If the spray controller's snapshot already lives where we want
            # it (rare), skip the copy.
            if os.path.realpath(src) != os.path.realpath(target):
                shutil.copy2(src, target)
            return target
        except Exception as e:
            logger.warning(f"Snapshot adopt/capture for {name} failed: {e}")
            return None

    # ------------------------------------------------------------------ #
    # Video recording (ffmpeg subprocess pulling RTSP)
    # ------------------------------------------------------------------ #
    def _start_recording(self, sess_dir: str) -> Optional[str]:
        if not shutil.which("ffmpeg"):
            logger.info("ffmpeg not found — skipping spray video recording")
            return None

        out_path = os.path.join(sess_dir, "spray.mp4")
        cmd = [
            "ffmpeg", "-y", "-rtsp_transport", "tcp",
            "-i", DEFAULT_RTSP_URL,
            "-c:v", "copy",         # passthrough — no re-encode
            "-an", "-f", "mp4", out_path,
        ]
        try:
            self._ffmpeg_proc = subprocess.Popen(
                cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                stdin=subprocess.PIPE,
            )
            return out_path
        except Exception as e:
            logger.warning(f"Failed to start ffmpeg recording: {e}")
            self._ffmpeg_proc = None
            return None

    def _stop_recording(self) -> None:
        proc = self._ffmpeg_proc
        if proc is None:
            return
        try:
            # Send 'q' on stdin so ffmpeg flushes the moov atom cleanly.
            try:
                if proc.stdin:
                    proc.stdin.write(b"q")
                    proc.stdin.flush()
                    proc.stdin.close()
            except Exception:
                pass
            proc.wait(timeout=4)
        except subprocess.TimeoutExpired:
            proc.terminate()
            try:
                proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                proc.kill()
        except Exception as e:
            logger.warning(f"ffmpeg shutdown error: {e}")
        finally:
            self._ffmpeg_proc = None


# Module-level singleton accessed from api.py
_singleton: Optional[SprayArtifactManager] = None


def get_artifact_manager() -> SprayArtifactManager:
    global _singleton
    if _singleton is None:
        _singleton = SprayArtifactManager()
    return _singleton
