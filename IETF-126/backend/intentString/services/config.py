LIMO_HOST = "192.168.50.165"
LIMO_ROSBRIDGE_PORT = 9090
LIMO_CAMERA_PORT = 8080

ROSBRIDGE_URL = f"ws://{LIMO_HOST}:{LIMO_ROSBRIDGE_PORT}"
SNAPSHOT_URL = f"http://{LIMO_HOST}:{LIMO_CAMERA_PORT}/snapshot"

OLLAMA_URL = "http://localhost:11434"
OLLAMA_MODEL = "llama3.1:8b"

YOLO_MODEL_PATH = "yolo11n.pt"
YOLO_CONFIDENCE = 0.5
FRAME_CENTER_THRESHOLD = 60  # pixels

# Safety e-stop — same bbox_height/frame_height proxy as yolo_trace.py
SAFETY_STOP_RATIO = 0.65      # ratio at/above which an obstacle forces e-stop
SAFETY_CHECK_INTERVAL = 0.3   # seconds between safety checks
