"""Canonical Cloud -> Edge intent validation; shared transport contract."""
import math
import time
from typing import Any, Dict
from common.config import ENV_CFG, MAP_CFG
from common.navigation_utils import grid_cell_to_map_xy
from common.speed_policy import validate_speed
VALID_ACTIONS = {"move", "find", "approach", "stop", "detour", "resume"}

def normalize_user_intent(raw: Dict[str, Any]) -> Dict[str, Any]:
    """
    Validates and normalizes Section 3 User Intent JSON into the canonical structure
    expected by Section 4 LLM Input (`intent`).
    """
    if not isinstance(raw, dict):
        raise ValueError("Intent must be an object")
    intent_id = str(raw.get("intent_id") or raw.get("id") or f"intent_{int(time.time())}")
    action = str(raw.get("action", "move")).lower().strip()
    if action not in VALID_ACTIONS:
        raise ValueError(f"Unsupported intent action '{action}'. Allowed: {sorted(VALID_ACTIONS)}")

    target_location = raw.get("target_location")
    normalized_loc: Any = None

    if isinstance(target_location, dict):
        if "cell_x" in target_location and "cell_y" in target_location and "x" not in target_location:
            mx, my = grid_cell_to_map_xy(int(target_location["cell_x"]), int(target_location["cell_y"]))
        else:
            mx, my = float(target_location["x"]), float(target_location["y"])
        normalized_loc = {
            "type": target_location.get("type", "pose"),
            "x": mx,
            "y": my,
            "frame_id": target_location.get("frame_id", ENV_CFG.planning_frame),
        }
    elif isinstance(target_location, (list, tuple)) and len(target_location) >= 2:
        normalized_loc = {
            "type": "pose",
            "x": float(target_location[0]),
            "y": float(target_location[1]),
            "frame_id": ENV_CFG.planning_frame,
        }
    elif isinstance(target_location, str) and target_location.strip():
        loc_name = target_location.strip()
        if ENV_CFG.map_available and loc_name in MAP_CFG.named_locations:
            coords = MAP_CFG.named_locations[loc_name]
            normalized_loc = {
                "type": "named_pose",
                "name": loc_name,
                "x": float(coords["x"]),
                "y": float(coords["y"]),
                "frame_id": "map",
            }
        else:
            normalized_loc = loc_name

    if isinstance(normalized_loc,dict):
        if not all(math.isfinite(normalized_loc[k]) for k in ("x","y")):
            raise ValueError("Coordinates must be finite")
        if normalized_loc["frame_id"] not in ("map","odom","base_link"):
            raise ValueError("Unsupported coordinate frame")
        if normalized_loc["frame_id"] == "map" and not ENV_CFG.map_available:
            raise ValueError("Map coordinates cannot be used in mapless mode")
    constraints = raw.get("constraints") or {}
    if not isinstance(constraints, dict):
        raise ValueError("constraints must be an object")
    if set(constraints) - {"stop_distance_m","safety_margin_m","avoid_obstacle_class","speed_mps"}:
        raise ValueError("Unsupported constraint; supported: stop_distance_m, safety_margin_m, avoid_obstacle_class, speed_mps")
    for key in ("stop_distance_m","safety_margin_m"):
        if key in constraints and (not isinstance(constraints[key],(int,float)) or not math.isfinite(constraints[key]) or constraints[key] < .30):
            raise ValueError(f"{key} must be finite and at least 0.30m")

    constraints = dict(constraints)
    if "speed_mps" in raw:
        if "speed_mps" in constraints and constraints["speed_mps"] != raw["speed_mps"]:
            raise ValueError("Conflicting speed_mps values")
        constraints["speed_mps"] = raw["speed_mps"]
    if "speed_mps" in constraints:
        constraints["speed_mps"] = validate_speed(constraints["speed_mps"])
    report = raw.get("report", False)
    if not isinstance(report, bool):
        raise ValueError("report must be true or false")
    return {
        "id": intent_id,
        "intent_id": intent_id,
        "action": action,
        "target_location": normalized_loc,
        "target_object": raw.get("target_object"),
        "constraints": constraints,
        "report": report,
        "priority": int(raw.get("priority", 1)),
        "timestamp": time.time(),
    }
