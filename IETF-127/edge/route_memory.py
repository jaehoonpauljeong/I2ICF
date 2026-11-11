"""Unresolved route constraints, not invented current obstacle observations.

A collision cannot be disproved by a short-radius query, no-return, occlusion,
or a safety-latch reset. A newer finite scan must positively see through its
location before removing it. New intent/frame/rewound simulation resets scope.
"""
import copy
import math
import threading
from common.runtime_safety import fresh, finite, valid_pose


class RouteMemory:
    def __init__(self):
        self.lock = threading.RLock()
        self.key = None
        self.entries = []
        self.seen = set()
        self.last_stamp = None

    def scope(self, world):
        key = (world.get('intent_version'), (world.get('intent') or {}).get('id') or
               (world.get('intent') or {}).get('intent_id'), world.get('robot_state', {}).get('frame_id') or (self.key[2] if self.key else None))
        stamp = world.get('raw_lidar', {}).get('stamp_sec')
        with self.lock:
            rewound = finite(stamp) and finite(self.last_stamp) and stamp < self.last_stamp - 1.
            if key != self.key or rewound:
                self.key, self.entries, self.seen = key, [], set()
            if finite(stamp):
                self.last_stamp = stamp

    def record(self, validation, frame, source_id, scan_stamp=None):
        if not isinstance(validation, dict) or validation.get('error_code') != 'PATH_COLLISION_DETECTED':
            return
        with self.lock:
            if source_id in self.seen:
                return
            self.seen.add(source_id)
            for hit in validation.get('collision_points', []):
                pos = hit.get('obstacle_pos') or {}
                hit_frame = hit.get('frame_id', validation.get('frame_id', frame))
                if hit_frame != frame or not all(finite(pos.get(k)) for k in ('x','y')):
                    continue
                if any(math.hypot(p['x']-pos['x'], p['y']-pos['y']) < .15 for p in self.entries):
                    continue
                self.entries.append({'x':pos['x'], 'y':pos['y'], 'frame_id':frame,
                    'source_id':str(source_id), 'scan_stamp_sec':scan_stamp,
                    'kind':'unresolved_route_constraint', 'live_observation':False})

    @staticmethod
    def positively_cleared(point, scan):
        tf = scan.get('laser_tf_in_planning_frame', {})
        if not fresh(scan) or not fresh(tf) or not tf.get('valid') or tf.get('frame_id') != point['frame_id']:
            return False
        stamp, original = scan.get('stamp_sec'), point.get('scan_stamp_sec')
        if not finite(stamp) or not finite(original) or stamp <= original:
            return False
        if not all(finite(tf.get(k)) for k in ('x','y','yaw')):
            return False
        dx,dy = point['x']-tf['x'], point['y']-tf['y']
        distance = math.hypot(dx,dy)
        angle = (math.atan2(dy,dx)-tf['yaw']+math.pi)%(2*math.pi)-math.pi
        inc, start = scan.get('angle_increment'), scan.get('angle_min')
        ranges = scan.get('ranges', [])
        if not ranges or not finite(inc) or inc <= 0 or not finite(start):
            return False
        index = round((angle-start)/inc)
        full = len(ranges)*inc >= 2*math.pi-2*inc
        # Three neighboring positive returns beyond the recorded collision, never -1/no-return.
        for offset in (-1,0,1):
            i = (index+offset)%len(ranges) if full else index+offset
            if not 0 <= i < len(ranges):
                return False
            r = ranges[i]
            mask = scan.get('self_detection_mask', [])
            if (i < len(mask) and mask[i]) or not finite(r) or r <= distance+.45:
                return False
            if not finite(scan.get('range_max')) or r > scan['range_max']:
                return False
        return True

    def snapshot(self, world):
        with self.lock:
            self.scope(world)
            nav = world.get('navigation', {})
            failure = nav.get('last_safety_stop') or {}
            if failure.get('intent_version') == world.get('intent_version'):
                self.record(failure.get('validation'), failure.get('frame_id'),
                            ('controller', failure.get('operation_id'), failure.get('timestamp')),
                            failure.get('scan_stamp_sec'))
            robot = world.get('robot_state', {})
            if valid_pose(robot):
                scan = world.get('raw_lidar', {})
                stamp = scan.get('stamp_sec')
                for point in self.entries:
                    if point.get('_last_checked_stamp') == stamp:
                        continue
                    point['_last_checked_stamp'] = stamp
                    point['_clear_count'] = point.get('_clear_count', 0)+1 if self.positively_cleared(point,scan) else 0
                self.entries = [p for p in self.entries if p.get('_clear_count',0) < 3]
            return [{k:copy.deepcopy(v) for k,v in p.items() if not k.startswith('_')} for p in self.entries]
