"""Resume only measured progress on the same absolute route and intent.

Never infer completion from nearest-point geometry: loops and detours may have
nearby, unvisited waypoints. Relative coordinates are new commands, not identities.
"""
import copy
from common.runtime_safety import finite


def remaining_waypoints(points, frame, navigation, version):
    result = {"matched_route":False, "skipped_waypoint_count":0,
              "points":copy.deepcopy(points)}
    path = navigation.get("current_path") or []
    index = navigation.get("current_wp_index")
    if (frame not in ("map", "odom") or frame != navigation.get("frame_id")
            or navigation.get("intent_version") != version
            or navigation.get("path_intent_version") != version
            or not isinstance(points, list) or not points or not path
            or type(index) is not int or not 0 <= index <= len(path)):
        return result

    def equal(a, b):
        return (isinstance(a, dict) and isinstance(b, dict)
                and set(a) == {"x", "y"} and set(b) == {"x", "y"}
                and all(finite(a[k]) and finite(b[k]) and abs(a[k]-b[k]) <= 1e-6 for k in ("x", "y")))

    # Full route or an exact suffix only. A revised route remains a new plan.
    offset = len(path)-len(points)
    if offset < 0 or not all(equal(a,b) for a,b in zip(points,path[offset:])):
        return result
    # A suffix that drops unvisited points must be independently treated as a new route.
    if offset > index:
        return result
    skipped = index-offset
    return {"matched_route":True, "skipped_waypoint_count":skipped,
            "resume_index":index, "source_operation_id":navigation.get("operation_id"),
            "points":copy.deepcopy(points[skipped:])}
