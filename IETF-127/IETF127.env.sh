# Source in EVERY application terminal on the execution computer, from the project directory.
# Profile for ViLaR's warehouse scene and the supplied limo_diff_thin.usd.
export ROBOCAR_ENV=isaac_sim
export MAP_AVAILABLE=true
export CAMERA_TOPIC=/sim/camera/color/image_raw
export CAMERA_INFO_TOPIC=/sim/camera/camera_info
export CAMERA_OPTICAL_FRAME=limo_camera_optical
export SCAN_TOPIC=/sim/scan_sync
export LIDAR_MAX_RANGE_M=30.0
export OBJECT_LIDAR_MAX_RANGE_M=12.0
export ODOM_TOPIC=/sim/odom
export CMD_VEL_TOPIC=/sim/cmd_vel
export BASE_FRAME=base_link
export LASER_FRAME=laser_link
# The default map and YOLO paths are resolved relative to the Python files.
# Keep STATIC_MAP_YAML / YOLO_MODEL unset unless intentionally overriding them.

# Simulation-only policy: -1 is unmeasured no-return, never a synthetic distance.
export ISAAC_ALLOW_NO_RETURN=true

# Original-height LIMO: rear self-occlusion remains UNKNOWN, never a synthetic return.
# Simulation ONLY: permit a slow swept-footprint check against the static map for these rays.
# This does not establish absence of dynamic obstacles in the blind region. Disabled on real robots.
export ISAAC_ALLOW_MAP_ONLY_REAR_SWEEP=true
