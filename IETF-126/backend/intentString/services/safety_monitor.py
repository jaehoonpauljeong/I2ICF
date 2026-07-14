"""
Safety Monitor — background YOLO loop, independent of detection/trace sessions.

Ported from IETF-125's imo_control.py distance e-stop, adapted to this
project's camera-only setup: instead of a LiDAR/ultrasonic distance reading,
proximity is estimated the same way yolo_trace.py already does — the ratio
of a detected box's height to the frame height. Whenever anything fills the
frame past SAFETY_STOP_RATIO, the shared RosBridge is forced to e-stop, which
overrides whatever action/trace/detection command is currently driving.
"""
import threading
import time

import cv2
import numpy as np
import requests

from .config import SNAPSHOT_URL, YOLO_MODEL_PATH, YOLO_CONFIDENCE, SAFETY_STOP_RATIO, SAFETY_CHECK_INTERVAL
from .rosbridge import bridge


class _SafetyMonitor:
    def __init__(self):
        t = threading.Thread(target=self._loop, daemon=True)
        t.start()

    def _loop(self) -> None:
        from ultralytics import YOLO
        model = YOLO(YOLO_MODEL_PATH)
        print("[Safety] Monitor started")

        while True:
            frame = _fetch_frame()
            if frame is None:
                time.sleep(SAFETY_CHECK_INTERVAL)
                continue

            too_close = False
            try:
                results = model(frame, conf=YOLO_CONFIDENCE, verbose=False)
                fh = frame.shape[0]
                for r in results:
                    for box in r.boxes:
                        _, y1, _, y2 = (int(v) for v in box.xyxy[0])
                        if (y2 - y1) / fh >= SAFETY_STOP_RATIO:
                            too_close = True
                            break
                    if too_close:
                        break
            except Exception as e:
                print(f"[Safety] YOLO error: {e}")

            bridge.set_estop(too_close)
            time.sleep(SAFETY_CHECK_INTERVAL)


def _fetch_frame():
    try:
        resp = requests.get(SNAPSHOT_URL, timeout=2)
        arr = np.frombuffer(resp.content, np.uint8)
        return cv2.imdecode(arr, cv2.IMREAD_COLOR)
    except Exception:
        return None


# Module-level singleton — starts monitoring immediately on import
safety_monitor = _SafetyMonitor()
