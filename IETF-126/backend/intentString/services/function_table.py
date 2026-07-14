"""
Function Table — maps action commands to (linear_x, angular_z).
Detection and trace modes are listed in the table as special entries.
"""
from typing import Optional

# (linear_x m/s, angular_z rad/s)
ACTION_TABLE: dict[str, tuple[float, float]] = {
    # straight / forward aliases
    "go straight":      ( 0.5,  0.0),
    "go straight slow": ( 0.2,  0.0),
    "go straight fast": ( 0.8,  0.0),
    "move forward":     ( 0.5,  0.0),
    "go forward":       ( 0.5,  0.0),
    "forward":          ( 0.5,  0.0),
    # reverse
    "move back":        (-0.5,  0.0),
    "move back slow":   (-0.2,  0.0),
    "move backward":    (-0.5,  0.0),
    "go back":          (-0.5,  0.0),
    # stop
    "stop":             ( 0.0,  0.0),
    # spin in place
    "rotate left":      ( 0.0,  0.6),
    "rotate right":     ( 0.0, -0.6),
    # arc turns
    "turn left":        ( 0.3,  0.5),
    "turn right":       ( 0.3, -0.5),
    # sharp turns
    "sharp left":       ( 0.2,  1.0),
    "sharp right":      ( 0.2, -1.0),
    # gentle curves
    "curve left":       ( 0.5,  0.3),
    "curve right":      ( 0.5, -0.3),
}

# YOLO-based modes — no cmd_vel, handled by dedicated sessions
YOLO_MODES = {"detection", "trace"}


def resolve(command: str, llm_speed: Optional[float]) -> tuple[float, float]:
    """
    Lookup command in table.
    If llm_speed provided and the command has non-zero linear_x,
    override magnitude (preserve direction sign) with llm_speed.
    Returns (linear_x, angular_z).
    """
    key = command.lower().replace("_", " ").strip()
    base_lx, az = ACTION_TABLE.get(key, (0.0, 0.0))

    if llm_speed is not None and base_lx != 0.0:
        linear_x = abs(llm_speed) * (1.0 if base_lx > 0 else -1.0)
    else:
        linear_x = base_lx

    return linear_x, az
