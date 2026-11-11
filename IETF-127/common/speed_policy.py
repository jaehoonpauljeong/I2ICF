"""Shared speed contract; user intent wins over model proposals, never over safety."""
import math

MIN_SPEED_MPS = 0.01
MAX_SPEED_MPS = 0.35
SPEED_SCHEMA = {"type": "number", "minimum": MIN_SPEED_MPS, "maximum": MAX_SPEED_MPS}


def validate_speed(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError("speed_mps must be a finite number")
    if not MIN_SPEED_MPS <= value <= MAX_SPEED_MPS:
        raise ValueError(f"speed_mps must be between {MIN_SPEED_MPS} and {MAX_SPEED_MPS} m/s")
    return float(value)


def select_speed(intent, proposed=None):
    user = (intent.get("constraints") or {}).get("speed_mps")
    if user is not None:
        return validate_speed(user), "user"
    if proposed is None:
        raise ValueError("SPEED_REQUIRED: choose speed_mps because the user did not specify one")
    return validate_speed(proposed), "vlm"
