"""Short-horizon swept-footprint checks; unknown rays are never converted to free ranges."""
import math
from common.config import ENV_CFG, LIDAR_CFG
from common.runtime_safety import fresh, finite, no_return_allowed


def check_motion(scan, pose, grid, linear, angular, allow_map_blind=False):
    if linear < 0:
        return {"ok":False,"error":"REVERSE_UNOBSERVED"}
    tf = scan.get("laser_to_base", {})
    if not tf.get("valid") or not fresh(tf):
        return {"ok":False,"error":"SCAN_TF_INVALID"}
    ranges = scan.get("ranges", [])
    increment = scan.get("angle_increment", 0.)
    if not ranges or not finite(increment) or abs(increment)<1e-9:
        return {"ok":False,"error":"MOTION_COVERAGE_UNKNOWN"}
    self_mask = scan.get("self_detection_mask", [])
    no_returns = scan.get("no_return_mask", [])
    xmin,xmax = LIDAR_CFG.robot_footprint_x_bounds
    ymin,ymax = LIDAR_CFG.robot_footprint_y_bounds
    # Include a 5cm border outside the configured chassis; sample at <=2.5cm.
    padding, step = .05, .025
    xs = [xmin-padding+i*(xmax-xmin+2*padding)/math.ceil((xmax-xmin+2*padding)/step)
          for i in range(math.ceil((xmax-xmin+2*padding)/step)+1)]
    ys = [ymin-padding+i*(ymax-ymin+2*padding)/math.ceil((ymax-ymin+2*padding)/step)
          for i in range(math.ceil((ymax-ymin+2*padding)/step)+1)]
    # Rechecked at every command: include half a second of motion, angular samples <=3 degrees.
    steps=max(1,math.ceil(abs(angular)*.5/math.radians(3)),math.ceil(abs(linear)*.5/step))
    visited=set()
    map_only=set()
    world_c,world_s=math.cos(pose['yaw']),math.sin(pose['yaw'])
    tc,ts=math.cos(tf['yaw']),math.sin(tf['yaw'])
    fallback=(allow_map_blind and ENV_CFG.env_mode=='isaac_sim' and ENV_CFG.map_available
              and pose.get('frame_id')=='map' and grid is not None and grid.available)
    for k in range(steps+1):
        t=.5*k/steps
        a=angular*t
        dx=linear*t if abs(angular)<1e-8 else linear*math.sin(a)/angular
        dy=0. if abs(angular)<1e-8 else linear*(1-math.cos(a))/angular
        c,s=math.cos(a),math.sin(a)
        for x in xs:
            for y in ys:
                bx,by=dx+c*x-s*y,dy+s*x+c*y
                if abs(angular)<1e-8 and linear>0 and bx<=xmax: continue
                if xmin<=bx<=xmax and ymin<=by<=ymax: continue  # Already occupied by chassis.
                key=(round(bx,3),round(by,3))
                if key in visited: continue
                visited.add(key)
                wx,wy=pose['x']+world_c*bx-world_s*by,pose['y']+world_s*bx+world_c*by
                if ENV_CFG.map_available:
                    if grid is None or not grid.available or pose.get('frame_id')!='map':
                        return {"ok":False,"error":"MAP_UNAVAILABLE"}
                    if not grid.clear(wx,wy,step):
                        return {"ok":False,"error":"MOTION_SWEEP_MAP_BLOCKED","point":{"x":wx,"y":wy}}
                lx,ly=tc*(bx-tf['x'])+ts*(by-tf['y']),-ts*(bx-tf['x'])+tc*(by-tf['y'])
                angle=math.atan2(ly,lx)
                candidates=[round((angle+turn*2*math.pi-scan['angle_min'])/increment) for turn in (-1,0,1)]
                indices=[i for i in candidates if 0<=i<len(ranges)]
                if not indices: return {"ok":False,"error":"MOTION_COVERAGE_UNKNOWN"}
                i=indices[0]
                distance=math.hypot(lx,ly)
                # Rearward is measured from the actual sensor origin along base_link's
                # forward axis. A forward-mounted LiDAR can self-occlude points at bx=0
                # (or slightly positive bx); these are still behind the LiDAR.
                # Check the selected ray too so angular-bin rounding cannot admit a
                # forward-looking ray. The sensor yaw may differ from base_link yaw.
                ray_forward=math.cos(scan['angle_min']+i*increment+tf['yaw'])
                behind_sensor=(bx < tf['x']-1e-9 and ray_forward < -1e-9)
                r=ranges[i]
                if finite(r) and r>0:
                    if r<distance+step:
                        return {"ok":False,"error":"MOTION_SWEEP_OBSTACLE","ray_index":i}
                elif (no_return_allowed(scan) and len(no_returns)==len(ranges) and no_returns[i] is True):
                    continue
                elif (fallback and behind_sensor and len(self_mask)==len(ranges) and self_mask[i] is True):
                    map_only.add(i)
                else:
                    self_marked=len(self_mask)==len(ranges) and self_mask[i] is True
                    no_return_marked=len(no_returns)==len(ranges) and no_returns[i] is True
                    reason=("map_only_rear_policy_unavailable" if not fallback else
                            "ray_or_point_not_behind_lidar_in_base_forward_axis" if not behind_sensor else
                            "self_detection_mask_missing_or_wrong_length" if len(self_mask)!=len(ranges) else
                            "ray_not_classified_as_self_detection")
                    return {"ok":False,"error":"MOTION_COVERAGE_UNKNOWN","ray_index":i,
                            "coverage_diagnostics":{
                                "fallback_rejection_reason":reason,
                                "sweep_time_sec":t,"point_in_base":{"x":bx,"y":by},
                                "point_in_laser":{"x":lx,"y":ly},"required_range_m":distance+step,
                                "ray_angle_rad":scan['angle_min']+i*increment,
                                "range_m":r if finite(r) else None,
                                "ray_classification":"self_detection" if self_marked else "no_return" if no_return_marked else "unclassified_invalid",
                                "ray_count":len(ranges),"self_detection_mask_length":len(self_mask),
                                "no_return_mask_length":len(no_returns),
                                "no_return_policy_allowed":no_return_allowed(scan),
                                "map_only_policy_eligible":bool(fallback),
                                "allow_map_blind":bool(allow_map_blind),
                                "rear_reference":"lidar_origin_base_forward_axis",
                                "point_behind_lidar":bx < tf['x']-1e-9,
                                "ray_forward_component":ray_forward,
                                "laser_to_base":tf,"scan_stamp_sec":scan.get('stamp_sec')}}
    return {"ok":True,"error":None,"map_only_blind_ray_count":len(map_only),
            "blind_region_policy":"static_map_only_simulation" if map_only else "observed",
            "dynamic_clearance_verified_in_blind_region":not bool(map_only),
            "rear_reference":"lidar_origin_base_forward_axis"}
