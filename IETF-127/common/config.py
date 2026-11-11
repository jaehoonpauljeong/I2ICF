#!/usr/bin/env python3
"""
IETF-127 Hackathon Project — Unified Configuration (`config.py`)
Combines:
- Base architecture from SDV_Robocar (IETF125)
- Isaac Sim ROS2 topics, multi-point LiDAR filtering, and Ollama(qwen) from SDV_Robocar (ICTC2026)
- TF2 coordinate hierarchy (map -> odom -> base_link -> laser_link), static map config,
  and robot self-detection filtering from ViLaR_IMO (semantic-dynamic-map)
- Model Context Protocol (MCP) Common ROS2 Execution Layer configuration
"""

import os
from pathlib import Path
from dataclasses import dataclass, field
from typing import Dict, Tuple


@dataclass
class EnvironmentConfig:
    # Execution mode: "isaac_sim" (Map-Based default) or "real_limo" (Mapless Local default)
    env_mode: str = os.getenv("ROBOCAR_ENV", "isaac_sim")  # "isaac_sim" | "real_limo"
    map_available: bool = (
        os.getenv(
            "MAP_AVAILABLE",
            "true" if os.getenv("ROBOCAR_ENV", "isaac_sim") == "isaac_sim" else "false",
        ).lower()
        == "true"
    )

    # Coordinate frame hierarchy (Section 2 of IETF-127 Design Plan)
    # Map-Based: map -> odom -> base_link -> laser_link
    # Mapless:   odom -> base_link -> laser_link
    global_frame: str = "map"
    odom_frame: str = "odom"
    base_frame: str = os.getenv("BASE_FRAME", "base_link")
    laser_frame: str = os.getenv("LASER_FRAME", "laser_link")
    # Optional calibrated optical origin; empty uses the image's ROS frame.
    camera_optical_frame: str = os.getenv("CAMERA_OPTICAL_FRAME", "")

    @property
    def planning_frame(self) -> str:
        """Returns 'map' when Static Map & Localization are active, else 'odom'."""
        return self.global_frame if self.map_available else self.odom_frame

    @property
    def navigation_mode(self) -> str:
        return "MAP_BASED" if self.map_available else "MAPLESS_LOCAL"


@dataclass
class RosTopicConfig:
    """
    ROS2 Topic mapping supporting both Isaac Sim (ICTC2026) and Real LIMO (IETF125).
    """

    env_mode: str = os.getenv("ROBOCAR_ENV", "isaac_sim")

    @property
    def scan_topic(self) -> str:
        return os.getenv(
            "SCAN_TOPIC", "/sim/scan" if self.env_mode == "isaac_sim" else "/scan"
        )

    @property
    def odom_topic(self) -> str:
        return os.getenv(
            "ODOM_TOPIC", "/sim/odom" if self.env_mode == "isaac_sim" else "/wheel/odom"
        )

    @property
    def camera_topic(self) -> str:
        return os.getenv(
            "CAMERA_TOPIC",
            (
                "/sim/camera/color/image_raw"
                if self.env_mode == "isaac_sim"
                else "/camera/color/image_raw"
            ),
        )

    @property
    def camera_info_topic(self) -> str:
        # ViLaR's ROS2Camera/cameraInfoHelper uses a different parent than RGB.
        return os.getenv(
            "CAMERA_INFO_TOPIC",
            (
                "/sim/camera/camera_info"
                if self.env_mode == "isaac_sim"
                else "/camera/color/camera_info"
            ),
        )

    @property
    def cmd_vel_topic(self) -> str:
        return os.getenv(
            "CMD_VEL_TOPIC",
            "/sim/cmd_vel" if self.env_mode == "isaac_sim" else "/cmd_vel",
        )

    # ViLaR_IMO (semantic-dynamic-map) semantic & dynamic map topics
    dynamic_object_detection_topic: str = "/dynamic_object_detection"
    dynamic_object_point_topic: str = "/dynamic_object"
    dynamic_object_marker_topic: str = "/dynamic_object_marker"
    semantic_object_topic: str = "/semantic_object"
    map_topic: str = "/map"
    planned_path_topic: str = "/planned_path"


@dataclass
class LidarFilterConfig:
    """
    LiDAR validity filtering (ICTC2026) + Robot Self-Detection Filtering (ViLaR_IMO).
    Prevents Isaac Sim invalid values (-1, 0.0, inf, NaN) and robot chassis reflections.
    """

    # ICTC2026 valid range bounds (strictly filters out -1, 0, inf, nan)
    min_valid_range_m: float = 0.05
    max_valid_range_m: float = float(os.getenv("LIDAR_MAX_RANGE_M", "30.0"))

    allow_no_return: bool = (
        os.getenv("ISAAC_ALLOW_NO_RETURN", "false").lower() == "true"
    )

    # ViLaR object association range; independent from the navigation scan range.
    object_min_range_m: float = 0.02
    object_max_range_m: float = float(os.getenv("OBJECT_LIDAR_MAX_RANGE_M", "12.0"))

    # Calibrated camera/LiDAR association in edge.edge_control
    short_group_k: int = (
        3  # Take median of k shortest valid rays to reject background bleed
    )

    # ViLaR_IMO (semantic-dynamic-map) Robot Self-Detection Filter
    # Points falling inside the LIMO chassis footprint in base_link/laser_link are rejected
    self_filter_radius_m: float = 0.25
    robot_footprint_x_bounds: Tuple[float, float] = (-0.22, 0.22)  # meters in base_link
    robot_footprint_y_bounds: Tuple[float, float] = (-0.16, 0.16)  # meters in base_link

    # TF2 lookup timeout (ViLaR_IMO scan_timestamp_sync & semantic_localization_node)
    tf_lookup_timeout_sec: float = 0.2
    max_sensor_age_sec: float = 1.5  # Stale data threshold for coordinate validity


@dataclass
class StaticMapConfig:
    """
    ViLaR_IMO (semantic-dynamic-map) static warehouse map configuration
    (`mapping/maps/static/static_map.yaml`).
    """

    map_yaml_path: str = os.getenv(
        "STATIC_MAP_YAML",
        str(
            Path(__file__).resolve().parents[1] / "mapping/maps/static/static_map.yaml"
        ),
    )
    image_file: str = "static_map.png"
    resolution: float = 0.05
    origin: Tuple[float, float, float] = (-27.975, -23.374999618530275, 0.0)
    occupied_thresh: float = 0.65
    free_thresh: float = 0.196

    # Named semantic locations in the map frame (resolvable from User Intent target_location)
    # OccupancyGrid reference cell for (0.0, 0.0) in full_warehouse+Limo+Object.usd
    origin_grid_cell: Tuple[int, int] = (559, 467)

    # Populate only with measured map coordinates for the deployed world.
    named_locations: Dict[str, Dict[str, float]] = field(default_factory=dict)


@dataclass
class OllamaLLMConfig:
    """
    ICTC2026 Ollama + qwen configuration extended for IETF-127 Dynamic Path Planning & MCP.
    """

    ollama_url: str = os.getenv("OLLAMA_URL", "http://localhost:11434/api/chat")
    model_name: str = os.getenv("OLLAMA_MODEL", "qwen3-vl:4b-instruct")
    temperature: float = 0.0
    num_predict: int = 2048
    num_ctx: int = int(os.getenv("OLLAMA_NUM_CTX", "8192"))
    # Bounded recovery for long tool conversations; never grow without a ceiling.
    max_num_ctx: int = int(os.getenv("OLLAMA_MAX_CTX", "16384"))
    request_timeout_sec: float = float(os.getenv("OLLAMA_TIMEOUT_SEC", "120"))
    native_tools: bool = os.getenv("OLLAMA_NATIVE_TOOLS", "false").lower() == "true"
    # Model turns, including observations and format repairs, not individual tool calls.
    max_mcp_tool_iterations: int = max(
        1, min(16, int(os.getenv("OLLAMA_MAX_TOOL_ITERATIONS", "8")))
    )


@dataclass
class NetworkEndpointsConfig:
    """
    Service ports and endpoints across Edge Server, Robot Streamer, Controller, and MCP Server.
    """

    imo_host: str = os.getenv("IMO_HOST", "127.0.0.1")
    cloud_host: str = os.getenv("CLOUD_HOST", "127.0.0.1")
    edge_host: str = os.getenv("EDGE_HOST", "127.0.0.1")

    # Sensor streamer (`imo_server_lidar.py`)
    sensor_port: int = int(os.getenv("SENSOR_PORT", "8000"))
    # ROS2 Path Executor & Controller (`imo_control.py`)
    control_port: int = int(os.getenv("CONTROL_PORT", "8001"))
    # Intent & Policy Server (`intent_server.py`)
    intent_port: int = int(os.getenv("INTENT_PORT", "5000"))
    # MCP ROS2 Server (`mcp_ros2_server.py`)
    mcp_port: int = int(os.getenv("MCP_PORT", "8090"))
    # Cloud / K8s Logging Server (`k8s_server.py`)
    k8s_port: int = int(os.getenv("K8S_PORT", "8080"))

    @property
    def sensor_base_url(self) -> str:
        return f"http://{self.imo_host}:{self.sensor_port}"

    @property
    def control_base_url(self) -> str:
        return f"http://{self.imo_host}:{self.control_port}"

    @property
    def intent_base_url(self) -> str:
        return f"http://{self.cloud_host}:{self.intent_port}"

    @property
    def mcp_base_url(self) -> str:
        return f"http://{self.edge_host}:{self.mcp_port}"


# Global singleton instances for easy import across modules
ENV_CFG = EnvironmentConfig()
ROS_TOPICS = RosTopicConfig(env_mode=ENV_CFG.env_mode)
LIDAR_CFG = LidarFilterConfig()
MAP_CFG = StaticMapConfig()
LLM_CFG = OllamaLLMConfig()
NET_CFG = NetworkEndpointsConfig()
# All services default to loopback; explicitly configure trusted LAN deployment.
BIND_HOST = os.getenv("BIND_HOST", "127.0.0.1")
WORLD_PORT = int(os.getenv("WORLD_PORT", "8002"))
WORLD_URL = os.getenv(
    "WORLD_URL", f"http://{NET_CFG.edge_host}:{WORLD_PORT}/world_state"
)
MIN_VALID_SCAN_RATIO = float(os.getenv("MIN_VALID_SCAN_RATIO", "0.1"))
MAX_SYNC_SKEW_SEC = float(os.getenv("MAX_SYNC_SKEW_SEC", "0.15"))
MAX_LOCAL_PATH_M = float(os.getenv("MAX_LOCAL_PATH_M", "2.5"))
MIN_CLEARANCE_M = 0.30
STOP_DISTANCE_M = 0.45

EDGE_BASE_URL = os.getenv("EDGE_BASE_URL", f"http://{NET_CFG.edge_host}:{WORLD_PORT}")
