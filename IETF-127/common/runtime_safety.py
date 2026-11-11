"""Shared freshness, exact-frame transforms and occupancy-map validation.

timestamp is acquisition/receipt wall time for cross-process freshness;
stamp_sec is the original ROS clock time and is used only for TF/synchronization.
Never replace a timestamp while returning a cached observation.
"""
import math
import time
from pathlib import Path
import numpy as np
import yaml
from PIL import Image
from common.config import LIDAR_CFG, MAP_CFG, MIN_VALID_SCAN_RATIO, ENV_CFG


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def fresh(data, now=None):
    now = time.time() if now is None else now
    stamp = data.get("timestamp") if isinstance(data, dict) else None
    return finite(stamp) and -0.05 <= now - stamp <= LIDAR_CFG.max_sensor_age_sec


def valid_pose(pose):
    return (fresh(pose) and pose.get("valid", pose.get("tf_valid", False)) is True and pose.get("tf_valid", True) is True
            and all(finite(pose.get(k)) for k in ("x", "y", "yaw"))
            and pose.get("frame_id") in ("map", "odom"))


def apply_tf(x, y, tf):
    if not fresh(tf) or not tf.get("valid"):
        return None
    if not all(finite(v) for v in (x, y, tf.get("x"), tf.get("y"), tf.get("yaw"))):
        return None
    c, s = math.cos(tf["yaw"]), math.sin(tf["yaw"])
    return {"x": tf["x"] + c*x - s*y, "y": tf["y"] + s*x + c*y,
            "frame_id": tf["frame_id"], "timestamp": tf["timestamp"], "unit": "m", "valid": True,
            **({"stamp_sec":tf["stamp_sec"]} if "stamp_sec" in tf else {})}


def no_return_allowed(scan):
    # Explicit agreement between sensor and consumer; no relaxation on real robots/mapless mode.
    return (LIDAR_CFG.allow_no_return and ENV_CFG.env_mode == "isaac_sim" and ENV_CFG.map_available
            and scan.get("no_return_policy") == "isaac_minus_one")


def scan_health(scan):
    if not fresh(scan) or not scan.get("valid", False):
        return "SCAN_STALE_OR_INVALID"
    total = scan.get("total_ray_count", 0)
    allowed = no_return_allowed(scan)
    mask = scan.get("no_return_mask", [])
    no_returns = sum(v is True for v in mask) if allowed and len(mask) == total else 0
    if total <= 0 or (scan.get("valid_ray_count", 0)+no_returns) / total < MIN_VALID_SCAN_RATIO:
        return "SCAN_INSUFFICIENT_RETURNS"
    if not scan.get("front_observed", False):
        front = scan.get("front_ray_count", 0)
        eligible = scan.get("front_valid_ray_count", 0)
        if allowed and no_returns:
            eligible += min(no_returns, scan.get("front_no_return_ray_count", 0))
        if not (allowed and front > 0 and eligible/front >= .5):
            return "SCAN_FRONT_UNKNOWN"
    return None


class OccupancyMap:
    """ROS trinary map: unknown and out-of-map cells are not traversable."""
    def __init__(self, yaml_path=None):
        self.available = False
        self.error = "MAP_NOT_LOADED"
        self.path = Path(yaml_path or MAP_CFG.map_yaml_path)
        try:
            cfg = yaml.safe_load(self.path.read_text())
            self.resolution = float(cfg["resolution"])
            self.origin = tuple(float(v) for v in cfg["origin"])
            if self.resolution <= 0 or len(self.origin) != 3 or not all(math.isfinite(v) for v in (*self.origin, self.resolution)):
                raise ValueError("Invalid map geometry")
            with Image.open(self.path.parent / cfg["image"]) as image:
                rgba = np.asarray(image.convert("RGBA"))
            gray = rgba[:, :, :3].mean(axis=2) / 255.0
            occupancy = gray if cfg.get("negate", 0) else 1.0 - gray
            # Only known free cells are traversable; keep unknown and occupied blocked.
            self.blocked = np.flipud((occupancy >= float(cfg["free_thresh"])) | (rgba[:, :, 3] < 255))
            self.height, self.width = self.blocked.shape
            self.available = True
            self.error = None
        except (OSError, ValueError, TypeError, KeyError, yaml.YAMLError) as exc:
            self.error = f"MAP_UNAVAILABLE: {exc}"

    def cell(self, x, y):
        dx, dy = x-self.origin[0], y-self.origin[1]
        c, s = math.cos(self.origin[2]), math.sin(self.origin[2])
        return math.floor((c*dx+s*dy)/self.resolution), math.floor((-s*dx+c*dy)/self.resolution)

    def cell_center(self, cx, cy):
        x, y = (cx+0.5)*self.resolution, (cy+0.5)*self.resolution
        c, s = math.cos(self.origin[2]), math.sin(self.origin[2])
        return self.origin[0]+c*x-s*y, self.origin[1]+s*x+c*y

    def clear(self, x, y, radius):
        if not self.available:
            return False
        cx, cy = self.cell(x, y)
        n = math.ceil(radius/self.resolution)
        if cx-n < 0 or cy-n < 0 or cx+n >= self.width or cy+n >= self.height:
            return False
        # Conservative square inflation includes the complete circular footprint.
        return not bool(self.blocked[cy-n:cy+n+1, cx-n:cx+n+1].any())

    def info(self):
        result = {"map_available": self.available, "error": self.error, "yaml_path": str(self.path)}
        if self.available:
            result.update(resolution_m=self.resolution, origin=list(self.origin), width=self.width, height=self.height)
        return result

    def preview(self, max_cells=40):
        """Bounded, conservative map overview for AI observation tools, not validation."""
        if not self.available:
            return None
        stride = max(1, math.ceil(max(self.width, self.height) / max_cells))
        rows = []
        for y in range(0, self.height, stride):
            rows.append("".join("#" if self.blocked[y:y+stride, x:x+stride].any() else "."
                                for x in range(0, self.width, stride)))
        return {"rows": rows, "row_order": "increasing_map_local_y", "origin": list(self.origin),
                "cell_size_m": self.resolution*stride,
                "legend": {"#": "occupied or unknown in at least one source cell", ".": "known free"},
                "note": "Coarse observation only; path validation uses the full-resolution map."}


def synchronized_stamps(first, second, max_skew):
    """Account only for sub-microsecond stamp rounding at the configured boundary."""
    return (finite(first) and finite(second) and first >= 0 and second >= 0
            and abs(first-second) <= max_skew + 1e-7)
