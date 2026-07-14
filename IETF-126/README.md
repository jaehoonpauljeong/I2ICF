## Main steps
**LIMO**<br/>
- Run rosbridge WebSocket server (exposes `/cmd_vel` etc. over WebSocket, port 9090)<br/>
- Run LIMO base driver (serial → wheel control, subscribes to `/cmd_vel`)<br/>
- Stream camera over HTTP (snapshot endpoint used by YOLO)<br/>

**Desktop (Django + Llama)**<br/>
- Receive natural-language text from the frontend (`POST /api/infer/`)<br/>
- Send the text as a prompt to a local Llama 3.1 8B (Ollama) instance<br/>
- Parse the LLM's JSON response into one of 3 modes: `action` / `trace` / `detection`<br/>
+ `action` → look up `(linear_x, angular_z)` in the function table, run for `duration` seconds<br/>
+ `trace` → start a YOLO loop that follows the requested object class (e.g. "person", "bottle")<br/>
+ `detection` → start a YOLO loop that searches for a person and runs a greeting motion<br/>
- Publish `geometry_msgs/Twist` messages to `/cmd_vel` over the rosbridge WebSocket connection<br/>
<br/>

### 1. Run rosbridge WebSocket server (LIMO)
```
$ ros2 launch rosbridge_server rosbridge_websocket_launch.xml    # terminal 1
```

### 2. Run LIMO base driver (LIMO)
```
$ ros2 launch limo_base limo_base.launch.py    # terminal 2
```

### 3. Run camera stream server (LIMO)
```
$ python3 ~/camera_stream.py    # terminal 3
```

### 4. Run Ollama with Llama 3.1 8B (Desktop)
```
$ ollama pull llama3.1:8b
$ ollama serve
```

### 5. Run Django backend (Desktop)
```
$ cd backend
$ python manage.py migrate
$ python manage.py runserver
```

### 6. Run React frontend (Desktop)
```
$ cd frontend
$ npm install
$ npm start
```

<br/>

## API

| Method | Path           | Description                                 |
|--------|----------------|----------------------------------------------|
| POST   | `/api/infer/`  | Send natural-language text, execute intent   |
| POST   | `/api/stop/`   | Stop all active sessions and the robot        |
| GET    | `/api/status/` | Current rosbridge/session status              |
| GET    | `/api/logs/`   | Recent intent and action logs                 |

<br/>

## Data classes
**yolo11n (COCO 80 classes) — used for `trace` and `detection` modes:**<br/>
{ 0: 'person',
 1: 'bicycle',
 2: 'car',
 3: 'motorcycle',
 4: 'airplane',
 5: 'bus',
 6: 'train',
 7: 'truck',
 8: 'boat',
 9: 'traffic light',
 10: 'fire hydrant',
 11: 'stop sign',
 12: 'parking meter',
 13: 'bench',
 14: 'bird',
 15: 'cat',
 16: 'dog',
 17: 'horse',
 18: 'sheep',
 19: 'cow',
 20: 'elephant',
 21: 'bear',
 22: 'zebra',
 23: 'giraffe',
 24: 'backpack',
 25: 'umbrella',
 26: 'handbag',
 27: 'tie',
 28: 'suitcase',
 29: 'frisbee',
 30: 'skis',
 31: 'snowboard',
 32: 'sports ball',
 33: 'kite',
 34: 'baseball bat',
 35: 'baseball glove',
 36: 'skateboard',
 37: 'surfboard',
 38: 'tennis racket',
 39: 'bottle',
 40: 'wine glass',
 41: 'cup',
 42: 'fork',
 43: 'knife',
 44: 'spoon',
 45: 'bowl',
 46: 'banana',
 47: 'apple',
 48: 'sandwich',
 49: 'orange',
 50: 'broccoli',
 51: 'carrot',
 52: 'hot dog',
 53: 'pizza',
 54: 'donut',
 55: 'cake',
 56: 'chair',
 57: 'couch',
 58: 'potted plant',
 59: 'bed',
 60: 'dining table',
 61: 'toilet',
 62: 'tv',
 63: 'laptop',
 64: 'mouse',
 65: 'remote',
 66: 'keyboard',
 67: 'cell phone',
 68: 'microwave',
 69: 'oven',
 70: 'toaster',
 71: 'sink',
 72: 'refrigerator',
 73: 'book',
 74: 'clock',
 75: 'vase',
 76: 'scissors',
 77: 'teddy bear',
 78: 'hair drier',
 79: 'toothbrush'
 }
<br/><br/>

## Structure
- `backend/` — Django REST API, LLM prompt/parsing, YOLO sessions, rosbridge client<br/>
- `frontend/` — single-page React control UI<br/>
