"""Measured final-pose completion shared by Edge and Robot; no model inference."""
import copy
import math
from common.runtime_safety import finite, valid_pose

GOAL_TOLERANCE_M = 0.15


def goal_evidence(pose, target):
    if not valid_pose(pose) or not isinstance(target, dict):
        return None
    if target.get('frame_id') != pose.get('frame_id'):
        return None
    if not all(finite(target.get(k)) for k in ('x', 'y')):
        return None
    distance = math.hypot(target['x']-pose['x'], target['y']-pose['y'])
    return {'within_tolerance': distance <= GOAL_TOLERANCE_M,
            'distance_m': distance, 'tolerance_m': GOAL_TOLERANCE_M,
            'target_location': copy.deepcopy(target), 'robot_pose': copy.deepcopy(pose)}


def measured_stopped(pose):
    """Require measured differential-drive velocities, not merely zero commands."""
    twist = pose.get('twist') or {}
    linear = (twist.get('linear') or {}).get('x')
    angular = (twist.get('angular') or {}).get('z')
    return (valid_pose(pose) and finite(linear) and finite(angular)
            and abs(linear) <= 0.02 and abs(angular) <= 0.02)


def path_requires_motion(points, pose):
    """For an already validated absolute path, mirror tick's waypoint tolerance.

    Any point outside the arrival disk requires motion, including a deliberate
    loop ending at the current position. This is not a final-goal success test.
    """
    return any(math.hypot(p['x']-pose['x'], p['y']-pose['y']) > GOAL_TOLERANCE_M for p in points)
