"""
Sequence Parser — builds ROS2-ready cmd_vel step list and executes via singleton RosBridge.
Uses the shared bridge connection to avoid dual-publisher conflicts on /cmd_vel.
"""
import time

from .rosbridge import bridge


def build_step(command: str, linear_x: float, angular_z: float, duration: float) -> dict:
    return {
        "command":   command,
        "linear_x":  linear_x,
        "angular_z": angular_z,
        "duration":  duration,
        "twist": {
            "linear":  {"x": linear_x, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0,      "y": 0.0, "z": angular_z},
        },
    }


def execute(steps: list[dict]) -> list[dict]:
    """
    Execute all steps via the singleton bridge at 10 Hz per step duration.
    """
    results = []
    try:
        bridge.ensure_connected()

        for step in steps:
            lx  = float(step["linear_x"])
            az  = float(step["angular_z"])
            end = time.time() + step["duration"]

            while time.time() < end:
                bridge.publish_cmd_vel(lx, az)
                time.sleep(0.1)

            bridge.stop_robot()
            time.sleep(0.1)

            print(f"[SeqParser] {step['command']} done (lx={lx}, az={az}, dur={step['duration']}s)")
            results.append({"command": step["command"], "success": True})

    except Exception as e:
        print(f"[SeqParser] Error: {e}")
        for step in steps[len(results):]:
            results.append({"command": step["command"], "success": False, "error": str(e)})

    return results
