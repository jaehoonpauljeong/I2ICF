#!/usr/bin/env python3
"""Shared pose math, robot self-filtering and collision-checked path planning.

Sensor ingestion lives in robot_car.imo_server_lidar; calibrated image/LiDAR
association lives in edge.edge_control. All paths use measured frame transforms.
"""
import math
from typing import Any, Dict, List, Optional, Tuple

from common.config import ENV_CFG, LIDAR_CFG, MIN_CLEARANCE_M, MAX_LOCAL_PATH_M
from common.runtime_safety import OccupancyMap, fresh, valid_pose, finite, apply_tf, scan_health


# ==============================================================================
# 1. Quaternion & 2D Pose Math (ViLaR_IMO TF2 conventions)
# ==============================================================================


def grid_cell_to_map_xy(cell_x: int, cell_y: int) -> Tuple[float, float]:
    """
    Converts OccupancyGrid cell indices `(cell_x, cell_y)` to ROS `map` / Isaac Sim World `(x, y)` meters
    using `static_map.yaml` (`resolution=0.05`, `origin=[-27.975, -23.374999618530275]`,
    where cell `(559, 467)` corresponds to `(0.0, 0.0)`):
      - Cell (559, 467) -> (0.0, 0.0)
      - Cell (259, 467) -> (-15.0, 0.0) [Person at (-14.99637, 0.00588)]
      - Cell (239, 467) -> (-16.0, 0.0) [Target Goal]
    """
    grid = OccupancyMap()
    if not grid.available:
        raise ValueError(grid.error)
    return grid.cell_center(int(cell_x), int(cell_y))


def yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    """Convert quaternion (x, y, z, w) to Euler yaw in radians [-pi, pi]."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle_rad: float) -> float:
    """Wrap angle into [-pi, pi]."""
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


# ==============================================================================
# 2. Robot Self-Detection and Range Validation
# ==============================================================================

def is_self_detection(x_laser: float, y_laser: float) -> bool:
    """
    ViLaR_IMO (semantic-dynamic-map) Robot Self-Detection Filter:
    Rejects LiDAR returns or object projections that fall inside the LIMO chassis footprint
    or within `self_filter_radius_m` of `laser_link` / `base_link`.
    """
    dist = math.hypot(x_laser, y_laser)
    if dist < LIDAR_CFG.self_filter_radius_m:
        return True
    xmin, xmax = LIDAR_CFG.robot_footprint_x_bounds
    ymin, ymax = LIDAR_CFG.robot_footprint_y_bounds
    if (xmin <= x_laser <= xmax) and (ymin <= y_laser <= ymax):
        return True
    return False


def is_valid_lidar_range(r: Optional[float]) -> bool:
    """
    ICTC2026 LiDAR invalid value filter:
    Isaac Sim frequently emits -1.0, 0.0, inf, or NaN for out-of-range rays.
    Returns True only if `r` is finite and strictly within [min_valid_range_m, max_valid_range_m].
    """
    if r is None:
        return False
    try:
        val = float(r)
    except (ValueError, TypeError):
        return False
    if not math.isfinite(val) or val <= 0.0:
        # Explicitly rejects -1, 0, inf, -inf, NaN
        return False
    return LIDAR_CFG.min_valid_range_m <= val < LIDAR_CFG.max_valid_range_m


# ==============================================================================
# 3. Path Validation and Detour Planning
# ==============================================================================


def validate_and_score_path(waypoints, path_frame_id, robot_state, detected_objects,
                            local_obstacle_points, safety_margin_m=0.45, *,
                            lidar_data=None, occupancy_map=None, route_constraints=None):
    def reject(code, reason, collisions=None):
        return {"valid": False, "error_code": code, "reason": reason, "frame_id":robot_state.get("frame_id"), "collision_points": collisions or []}
    if not valid_pose(robot_state):
        return reject("LOCALIZATION_INVALID", "Position is missing, invalid, or stale")
    robot_frame = robot_state["frame_id"]
    if path_frame_id not in (robot_frame, "base_link"):
        return reject("FRAME_MISMATCH", "No verified transform for path frame")
    if path_frame_id == "map" and not ENV_CFG.map_available:
        return reject("INVALID_FRAME_MAPLESS", "map frame is unavailable")
    if not finite(safety_margin_m) or safety_margin_m < MIN_CLEARANCE_M:
        return reject("INVALID_CLEARANCE", "Clearance is below the robot safety minimum")
    scan = lidar_data or {}
    error = scan_health(scan)
    if error:
        return reject(error, "Fresh observed LiDAR coverage is required")
    if not waypoints or len(waypoints) > 128:
        return reject("INVALID_PATH", "Provide 1 to 128 finite waypoints")
    if any(not isinstance(p, dict) or not all(finite(p.get(k)) for k in ("x", "y")) for p in waypoints):
        return reject("INVALID_PATH", "Non-finite or missing coordinate")
    points = [dict(p) for p in waypoints]
    if path_frame_id == "base_link":
        points = [apply_tf(p["x"], p["y"], robot_state) for p in points]
    grid = occupancy_map
    if ENV_CFG.map_available:
        grid = grid or OccupancyMap()
        if not grid.available or robot_frame != "map":
            return reject("MAP_UNAVAILABLE", "A loaded map and map localization are required")
    elif robot_frame != "odom":
        return reject("INVALID_FRAME_MAPLESS", "Local navigation requires odom localization")
    tf = scan.get("laser_tf_in_planning_frame", {})
    if not fresh(tf) or not tf.get("valid") or tf.get("frame_id") != robot_frame:
        return reject("SCAN_TF_INVALID", "Scan-time laser transform is unavailable")
    obstacles = []
    # Conservatively retain known blocked locations until positive clearance is observed.
    # These are planning constraints; do not timestamp them as fresh detections.
    for point in route_constraints or []:
        if point.get("frame_id") == robot_frame and all(finite(point.get(k)) for k in ("x","y")):
            obstacles.append((point["x"],point["y"],"unresolved_route_constraint"))
    for obj in detected_objects:
        pos = obj.get("position") or {}
        if fresh(pos) and pos.get("valid") and pos.get("frame_id") == robot_frame:
            if all(finite(pos.get(k)) for k in ("x", "y")):
                obstacles.append((pos["x"], pos["y"], obj.get("id", "object")))
    # Semantic detections supplement, never replace, every valid LiDAR return.
    for pt in local_obstacle_points:
        world = apply_tf(pt["x_laser"], pt["y_laser"], tf)
        if world:
            obstacles.append((world["x"], world["y"], "lidar_obstacle"))
    curr_x, curr_y = robot_state["x"], robot_state["y"]
    total = 0.
    for index, wp in enumerate(points):
        length = math.hypot(wp["x"]-curr_x, wp["y"]-curr_y)
        total += length
        if total > (100. if ENV_CFG.map_available else MAX_LOCAL_PATH_M):
            return reject("PATH_HORIZON_EXCEEDED", "Use shorter segments and observe again")
        steps = max(1, math.ceil(length / (min(0.05, grid.resolution/2) if grid else 0.05)))
        for step in range(steps+1):
            x = curr_x + step/steps*(wp["x"]-curr_x)
            y = curr_y + step/steps*(wp["y"]-curr_y)
            if grid and not grid.clear(x, y, safety_margin_m):
                return reject("STATIC_MAP_COLLISION", "Path footprint overlaps occupied, unknown, or out-of-map cells")
            for ox, oy, label in obstacles:
                if math.hypot(x-ox, y-oy) < safety_margin_m:
                    return reject("PATH_COLLISION_DETECTED", "Path footprint intersects an obstacle",
                                  [{"waypoint_index": index, "obstacle": label, "frame_id":robot_frame,"obstacle_pos": {"x": ox, "y": oy},
                                    "distance_from_robot_m":math.hypot(ox-robot_state["x"],oy-robot_state["y"])}])
            if not ENV_CFG.map_available and math.hypot(x-robot_state["x"], y-robot_state["y"]) > MIN_CLEARANCE_M:
                dx, dy = x-tf["x"], y-tf["y"]
                angle = normalize_angle(math.atan2(dy, dx)-tf["yaw"])
                inc = scan.get("angle_increment", 0.)
                ranges = scan.get("ranges", [])
                if not finite(inc) or abs(inc) < 1e-9:
                    return reject("SCAN_GEOMETRY_INVALID", "Invalid angular increment")
                ray = round((angle-scan["angle_min"])/inc)
                if abs(inc)*len(ranges) >= 2*math.pi-2*abs(inc):
                    ray %= len(ranges)
                if not 0 <= ray < len(ranges) or not is_valid_lidar_range(ranges[ray]) or ranges[ray] < math.hypot(dx,dy)+safety_margin_m:
                    return reject("LOCAL_SPACE_UNOBSERVED", "Local path leaves observed free space")
        curr_x, curr_y = wp["x"], wp["y"]
    return {"valid": True, "error_code": None, "reason": "Validated against fresh scan and map",
            "collision_points": [], "path_length_m": total, "points": points, "frame_id": robot_frame}


def generate_candidate_detour_path(
    start_x: float,
    start_y: float,
    goal_x: float,
    goal_y: float,
    frame_id: str,
    robot_state: Dict[str, Any],
    detected_objects: List[Dict[str, Any]],
    local_obstacle_points: List[Dict[str, float]],
    safety_margin_m: float = 0.45,
    lidar_data=None, occupancy_map=None, route_constraints=None,
) -> Dict[str, Any]:
    """
    Helper for MCP `plan_path` tool:
    Checks direct path first; if blocked by an obstacle, generates left/right lateral detour waypoints
    and validates each candidate against obstacles.
    """
    direct_wps = [{"x": round(goal_x, 3), "y": round(goal_y, 3)}]
    direct_check = validate_and_score_path(
        direct_wps, frame_id, robot_state, detected_objects, local_obstacle_points, safety_margin_m, lidar_data=lidar_data, occupancy_map=occupancy_map, route_constraints=route_constraints
    )
    audit = {"algorithm":"direct_then_first_valid_lateral_detour", "selection_rule":"first valid, not shortest",
             "start":{"x":start_x,"y":start_y}, "goal":{"x":goal_x,"y":goal_y},
             "effective_safety_margin_m":safety_margin_m, "frame_id":frame_id,
             "candidates":[{"kind":"direct","points":direct_wps,"validation":direct_check}],
             "selected_candidate":None}
    if direct_check["valid"]:
        audit["selected_candidate"]=0
        return {
            "success": True,
            "action": "move",
            "frame_id": frame_id,
            "path": {"type": "waypoints", "points": direct_wps},
            "validation": direct_check,
            "audit_details": audit,
        }

    # Compute perpendicular unit vector to the start->goal line for detour candidates
    dx = goal_x - start_x
    dy = goal_y - start_y
    dist = max(0.1, math.hypot(dx, dy))
    ux, uy = dx / dist, dy / dist
    px, py = -uy, ux  # Left perpendicular vector

    mid_x = start_x + 0.5 * dx
    mid_y = start_y + 0.5 * dy

    # Identify primary blocking obstacle coordinates from direct_check
    first_col = direct_check["collision_points"][0]["obstacle_pos"] if direct_check.get("collision_points") else {"x": mid_x, "y": mid_y}
    obs_x, obs_y = float(first_col["x"]), float(first_col["y"])
    if frame_id == "base_link" and direct_check.get("collision_points"):
        ox,oy = obs_x-robot_state["x"],obs_y-robot_state["y"]
        yaw=robot_state["yaw"]
        obs_x,obs_y = math.cos(yaw)*ox+math.sin(yaw)*oy,-math.sin(yaw)*ox+math.cos(yaw)*oy
    obs_proj = max(0.0, min(dist, (obs_x - start_x) * ux + (obs_y - start_y) * uy))

    # Build smooth corridor approach + local obstacle bypass + goal convergence waypoints
    pre_dist = max(0.0, obs_proj - 2.0)
    pass_start_dist = max(0.0, obs_proj - 1.0)
    pass_end_dist = min(dist, obs_proj + 0.85)

    audit["geometry"]={"blocking_obstacle":first_col,"obstacle_projection_m":obs_proj,
                       "approach_distance_m":pre_dist,"pass_start_m":pass_start_dist,"pass_end_m":pass_end_dist}
    audit["offset_order_m"]=[.85,-.85,1.25,-1.25,1.65,-1.65]
    # Try lateral offsets (+0.85m, -0.85m, +1.25m, -1.25m, +1.65m, -1.65m)
    for offset in (0.85, -0.85, 1.25, -1.25, 1.65, -1.65):
        candidate = []
        if pre_dist > 1.0:
            candidate.append({
                "x": round(start_x + pre_dist * ux, 3),
                "y": round(start_y + pre_dist * uy, 3),
            })
        candidate.extend([
            {
                "x": round(start_x + pass_start_dist * ux + offset * px, 3),
                "y": round(start_y + pass_start_dist * uy + offset * py, 3),
            },
            {
                "x": round(start_x + pass_end_dist * ux + offset * px, 3),
                "y": round(start_y + pass_end_dist * uy + offset * py, 3),
            },
            {"x": round(goal_x, 3), "y": round(goal_y, 3)},
        ])
        check = validate_and_score_path(
            candidate, frame_id, robot_state, detected_objects, local_obstacle_points, safety_margin_m, lidar_data=lidar_data, occupancy_map=occupancy_map, route_constraints=route_constraints
        )
        audit["candidates"].append({"kind":"detour","offset_m":offset,"points":candidate,"validation":check})
        if check["valid"]:
            audit["selected_candidate"]=len(audit["candidates"])-1
            return {
                "success": True,
                "action": "detour",
                "frame_id": frame_id,
                "path": {"type": "waypoints", "points": candidate},
                "validation": check,
                "audit_details": audit,
            }

    return {
        "success": False,
        "action": "stop",
        "frame_id": frame_id,
        "path": {"type": "waypoints", "points": []},
        "validation": direct_check,
            "audit_details": audit,
    }

def goal_geometry(robot, target):
    """Measured goal direction in one verified frame; not an AI motion decision."""
    if not valid_pose(robot) or not isinstance(target, dict) or target.get('frame_id') != robot.get('frame_id'):
        return None
    if not all(finite(target.get(k)) for k in ('x','y')):
        return None
    dx,dy = target['x']-robot['x'], target['y']-robot['y']
    bearing = math.atan2(dy, dx)
    error = normalize_angle(bearing-robot['yaw'])
    return {'frame_id':robot['frame_id'], 'distance_m':math.hypot(dx,dy),
            'goal_bearing_deg':math.degrees(bearing),'robot_yaw_deg':math.degrees(robot['yaw']),
            'heading_error_deg':math.degrees(error), 'controller_aligns_path_automatically':True,
            'angle_convention':'relative counterclockwise; -180 and +180 degrees are the same heading'}
