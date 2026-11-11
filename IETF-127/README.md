# IETF-127 MCP-Robotics

Last updated: **2026-11-10**

A navigation framework that accepts user intents and uses a **VLM to select tools and paths for controlling a ROS2 robot**.

## Architecture

| Layer | Modules | Role |
|---|---|---|
| Cloud | `manager/`, `cloud/` | Natural-language translation and intent delivery |
| Edge | `edge/` | YOLO–LiDAR fusion, VLM decisions, MCP tools and reports |
| Robot Car | `robot_car/`, `launch/` | ROS2 sensors, motion control and safety checks |

- Collision detection triggers a stop and requires replanning. Resumption preserves route progress and retains safety checks.
- User speed takes priority; otherwise the VLM selects a speed and records its explanation.
- Success requires measured arrival and verified stopped motion.

## Setup

**Isaac Sim 4.5 · ROS2 Humble · Python 3.10 · Ollama · `qwen3-vl:4b-instruct`**

Clone the repository, then run the following commands from the project root containing `requirements.txt` and `IETF127.env.sh`.

```bash
source /opt/ros/humble/setup.bash
python3 -m pip install -r requirements.txt
ollama pull qwen3-vl:4b-instruct
```

Keep Ollama running. Assets: `mapping/maps/isaac/full_warehouse_limo.usda`, `mapping/maps/static/static_map.yaml` and `static_map.png`, and `detector/yolov8s.pt`.

## Run

Open the scene in Isaac Sim, enable ROS2 Bridge and start playback.

In **each terminal**, enter the project root and run:

```bash
source /opt/ros/humble/setup.bash
source ./IETF127.env.sh
export OLLAMA_MODEL=qwen3-vl:4b-instruct
```

Start each service in a separate terminal:

| Terminal | Command | Service / port |
|---|---|---|
| T1 | `ros2 launch ./launch/vilar_frames.launch.py` | TF / scan sync |
| T2 | `python3 -m robot_car.imo_server_lidar` | Sensor API · 8000 |
| T3 | `python3 -m robot_car.imo_control` | Robot controller · 8001 |
| T4 | `python3 -m cloud.intent_server` | Intent API · 5000 |
| T5 | `python3 -m edge.edge_control` | Edge / MCP client · 8002 |
| T6 | `python3 -m edge.mcp_ros2_server` | MCP server · 8090 |


Before sending an intent:

```bash
curl -sS http://127.0.0.1:8002/health
python3 -m edge.mcp_client
```

Confirm `pose_valid:true`, `perception.ready:true`, `scan_error:null`, `map.map_available:true`, and a successful MCP probe listing 12 tools.

## Commands and Reports

Run **one** command for the intended speed mode:

```bash
python3 -m manager.run "Go to x -16, y 0 and report back" --send
python3 -m manager.run "Go to x -16, y 0 at 0.2 m/s and report back" --send
```

Manager supports numeric x/y coordinate movement requests. Omit `--send` to preview and save the intent without execution. Speed is **0.01–0.35 m/s**; safety checks may slow or stop motion.

| Report preference | Intent | Edge `/health` |
|---|---|---|
| “Go and report back” | `report:true` | Includes `requested_report` |
| “Go, but do not report back” | `report:false` | `requested_report:null` |

Logs, T5 terminal results and manual reports remain available with `false`. Report preference is model-interpreted; the default and model-failure fallback are `true`.

Read progress and results; replace `move_001` with the actual intent ID:

```bash
curl -sS http://127.0.0.1:8001/navigation/status
curl -sS 'http://127.0.0.1:8002/reports?intent_id=move_001'
```

Reports include completion time, target/current positions and success or failure. Request acceptance is not navigation completion.

To stop:

```bash
curl -sS -X POST http://127.0.0.1:5000/receive_intent \
  -H 'Content-Type: application/json' \
  -d '{"intent_id":"stop_001","action":"stop"}'
```

## Logs and Validation

One JSON is saved per scenario at termination under `logs/scenarios/`; T5 prints the final result. The filename uses Edge process start time and a sequence number. `intent_id` and `intent_version` connect planning, stops, recovery and completion. Logs include model responses, MCP calls, path validation and speed explanations. Active logs are buffered in memory and can be lost on a hard process termination.

Latest supplied Isaac Sim log: **`20261111_134306_252218_0001.json`** (`move_001`).

| Item | Result |
|---|---|
| Outcome | Success; stop verified and Robot intent closed |
| Target / final position | `(-16.0, 0.0) / (-16.0366, -0.1277)` in map |
| Goal error / duration | **13.9 cm / 10 min 23 sec** |
| Planning / speed | Collision detected → 4-waypoint detour; VLM selected **0.2 m/s** with explanation
| Model / MCP | **10 model responses / 9 actual tools/call requests** (`stop_robot` ×2, `plan_path` ×2, `execute_path` ×4, `reset_safety_stop` ×1); 2 motion operations accepted |
| Completion | **100%**, no remaining path |

This records one run, not a performance benchmark.

Cloud–Edge communication uses HTTP JSON. The standard A2A protocol has not yet been implemented.
