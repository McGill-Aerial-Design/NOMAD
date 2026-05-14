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
