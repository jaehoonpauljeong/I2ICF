"""
Trace session (YOLO loop).

Loop:
  1. Grab frame from LIMO camera snapshot
  2. YOLO inference → find target class
  3. Compute frame-adjust command (center horizontally, maintain distance)
  4. Publish cmd_vel via shared RosBridge
  5. Repeat until stopped
"""
import threading
import time
from typing import Optional

import cv2
import numpy as np
import requests

from .config import SNAPSHOT_URL, YOLO_MODEL_PATH, YOLO_CONFIDENCE, FRAME_CENTER_THRESHOLD
from .rosbridge import bridge

_SEARCH_AZ = 0.3       # rad/s when target not visible
_CLOSE_RATIO = 0.65    # bbox_height/frame_height → too close
_FAR_RATIO = 0.25      # bbox_height/frame_height → too far
_APPROACH_SPEED = 0.3  # m/s toward target
_BACKOFF_SPEED = 0.2   # m/s away from target
_TURN_GAIN = 0.3       # rad/s per lateral error unit


class TraceSession:
    def __init__(self):
        self._running = False
        self._target_class = "person"
        self._thread: Optional[threading.Thread] = None

    @property
    def running(self) -> bool:
        return self._running

    def start(self, target_class: str = "person") -> None:
        if self._running:
            self.stop()
        self._target_class = target_class
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        print(f"[Trace] Session started — target: {target_class}")

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=3)
        print("[Trace] Session stopped")

    # ── internal ──────────────────────────────────────────────────────────

    def _loop(self) -> None:
        try:
            from ultralytics import YOLO
            model = YOLO(YOLO_MODEL_PATH)
            bridge.ensure_connected()
            print(f"[Trace] Loop running — target: {self._target_class}")

            no_frame_count = 0
            while self._running:
                frame = _fetch_frame()
                if frame is None:
                    no_frame_count += 1
                    if no_frame_count % 10 == 1:
                        print(f"[Trace] Camera fetch failed ({no_frame_count}x) — {SNAPSHOT_URL}")
                    time.sleep(0.5)
                    continue
                no_frame_count = 0

                results = model(frame, conf=YOLO_CONFIDENCE, verbose=False)
                targets = [
                    box
                    for r in results
                    for box in r.boxes
                    if model.names[int(box.cls)] == self._target_class
                ]

                if not targets:
                    bridge.publish_cmd_vel(0.0, _SEARCH_AZ)
                    time.sleep(0.3)
                    continue

                # Best confidence target
                best = max(targets, key=lambda b: float(b.conf))
                x1, y1, x2, y2 = (int(v) for v in best.xyxy[0])
                cx = (x1 + x2) // 2
                bbox_h = y2 - y1
                fw, fh = frame.shape[1], frame.shape[0]

                offset = cx - fw // 2
                if abs(offset) > FRAME_CENTER_THRESHOLD:
                    angular_z = _TURN_GAIN if offset < 0 else -_TURN_GAIN
                else:
                    angular_z = 0.0

                ratio = bbox_h / fh
                if ratio < _FAR_RATIO:
                    linear_x = _APPROACH_SPEED
                elif ratio > _CLOSE_RATIO:
                    linear_x = -_BACKOFF_SPEED
                else:
                    linear_x = 0.0

                print(f"[Trace] target found — offset={offset} ratio={ratio:.2f} lx={linear_x} az={angular_z}")
                bridge.publish_cmd_vel(linear_x, angular_z)
                time.sleep(0.1)

        except Exception as e:
            print(f"[Trace] Loop error: {e}")
        finally:
            bridge.stop_robot()
            print("[Trace] Loop exited")


def _fetch_frame() -> Optional[np.ndarray]:
    try:
        resp = requests.get(SNAPSHOT_URL, timeout=2)
        arr = np.frombuffer(resp.content, np.uint8)
        return cv2.imdecode(arr, cv2.IMREAD_COLOR)
    except Exception:
        return None


# Module-level singleton
trace_session = TraceSession()
