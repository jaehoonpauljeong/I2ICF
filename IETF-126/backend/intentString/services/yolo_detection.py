"""
Detection + Greet session (YOLO loop).

Loop:
  1. Grab frame from LIMO camera snapshot
  2. YOLO inference → detect person
  3. Person found → execute greet sequence via shared RosBridge
  4. No person  → slow rotation to search
  5. Repeat until stopped
"""
import threading
import time
from typing import Optional

import cv2
import numpy as np
import requests

from .config import SNAPSHOT_URL, YOLO_MODEL_PATH, YOLO_CONFIDENCE
from .rosbridge import bridge

# Greet choreography: (linear_x, angular_z, duration_sec)
_GREET = [
    (0.0,  0.6, 0.4),   # turn left
    (0.0, -0.6, 0.8),   # turn right
    (0.0,  0.6, 0.4),   # turn left (back to center)
    (0.0,  0.0, 0.5),   # pause
]

_SEARCH_AZ = 0.3   # rad/s search rotation
_SEARCH_STEP = 0.3  # seconds per search tick


class DetectionSession:
    def __init__(self):
        self._running = False
        self._thread: Optional[threading.Thread] = None

    @property
    def running(self) -> bool:
        return self._running

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        print("[Detection] Session started")

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=3)
        print("[Detection] Session stopped")

    # ── internal ──────────────────────────────────────────────────────────

    def _loop(self) -> None:
        from ultralytics import YOLO
        model = YOLO(YOLO_MODEL_PATH)
        bridge.ensure_connected()

        while self._running:
            frame = _fetch_frame()
            if frame is None:
                time.sleep(0.5)
                continue

            results = model(frame, conf=YOLO_CONFIDENCE, verbose=False)
            person_found = any(
                model.names[int(box.cls)] == "person"
                for r in results for box in r.boxes
            )

            if person_found:
                self._greet()
            else:
                # Search: slow rotation
                bridge.publish_cmd_vel(0.0, _SEARCH_AZ)
                time.sleep(_SEARCH_STEP)

        bridge.stop_robot()

    def _greet(self) -> None:
        """Execute greeting choreography via shared RosBridge."""
        for lx, az, dur in _GREET:
            if not self._running:
                break
            end = time.time() + dur
            while time.time() < end and self._running:
                bridge.publish_cmd_vel(lx, az)
                time.sleep(0.1)
        bridge.stop_robot()
        # Pause before next detection cycle
        time.sleep(2.0)


def _fetch_frame() -> Optional[np.ndarray]:
    try:
        resp = requests.get(SNAPSHOT_URL, timeout=2)
        arr = np.frombuffer(resp.content, np.uint8)
        return cv2.imdecode(arr, cv2.IMREAD_COLOR)
    except Exception:
        return None


# Module-level singleton
detection_session = DetectionSession()
