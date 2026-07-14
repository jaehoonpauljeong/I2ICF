"""
Singleton RosBridge WebSocket session.
Shared by action execution, detection loop, and trace loop.
Auto-connects at startup and reconnects on disconnect.
"""
import json
import threading
import time

import websocket

from .config import ROSBRIDGE_URL

CMD_VEL_TOPIC   = "/cmd_vel"
CMD_VEL_TYPE    = "geometry_msgs/msg/Twist"
WAYPOINTS_TOPIC = "/astar_waypoints"
WAYPOINTS_TYPE  = "std_msgs/msg/String"

_STOP_MSG = {
    "linear":  {"x": 0.0, "y": 0.0, "z": 0.0},
    "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
}

_RECONNECT_INTERVAL = 5   # seconds between reconnect attempts
_CONNECT_TIMEOUT    = 3   # seconds for WebSocket connect


class _RosBridge:
    def __init__(self):
        self._ws        = None
        self._lock      = threading.Lock()
        self._connected = False
        # background reconnection thread
        t = threading.Thread(target=self._reconnect_loop, daemon=True)
        t.start()

    @property
    def connected(self) -> bool:
        return self._connected

    def ensure_connected(self) -> None:
        """Blocking connect (called from command path)."""
        if self._connected:
            return
        self._try_connect()

    def publish_cmd_vel(self, linear_x: float, angular_z: float) -> None:
        msg = {
            "linear":  {"x": float(linear_x), "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": float(angular_z)},
        }
        payload = {"op": "publish", "topic": CMD_VEL_TOPIC, "msg": msg}
        try:
            self.ensure_connected()
            self._send(payload)
        except Exception:
            # Stale socket — reconnect once and retry
            if self._try_connect():
                self._send(payload)
            else:
                raise

    def stop_robot(self) -> None:
        try:
            self._send({"op": "publish", "topic": CMD_VEL_TOPIC, "msg": _STOP_MSG})
        except Exception:
            pass

    def disconnect(self) -> None:
        if self._ws:
            try:
                self._ws.close()
            except Exception:
                pass
        self._connected = False

    # ── internal ──────────────────────────────────────────────────────────

    def _try_connect(self) -> bool:
        try:
            ws = websocket.create_connection(ROSBRIDGE_URL, timeout=_CONNECT_TIMEOUT)
            with self._lock:
                self._ws = ws
                self._connected = True
            self._advertise(CMD_VEL_TOPIC,   CMD_VEL_TYPE)
            self._advertise(WAYPOINTS_TOPIC, WAYPOINTS_TYPE)
            print(f"[RosBridge] Connected → {ROSBRIDGE_URL}")
            return True
        except Exception as e:
            self._connected = False
            print(f"[RosBridge] Connection failed: {e}")
            return False

    def _reconnect_loop(self) -> None:
        """Continuously try to (re)connect every _RECONNECT_INTERVAL seconds."""
        while True:
            if not self._connected:
                self._try_connect()
            time.sleep(_RECONNECT_INTERVAL)

    def _advertise(self, topic: str, msg_type: str) -> None:
        self._send({"op": "advertise", "topic": topic, "type": msg_type})
        time.sleep(0.1)

    def _send(self, payload: dict) -> None:
        with self._lock:
            try:
                self._ws.send(json.dumps(payload))
            except Exception:
                self._connected = False
                raise


# Module-level singleton — starts reconnect loop immediately
bridge = _RosBridge()
