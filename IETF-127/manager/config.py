import os

# Same variable names as the Edge side (common/config.py); /api/chat -> /api/generate is derived below.
OLLAMA_BASE_URL = os.getenv("OLLAMA_URL", "http://localhost:11434/api/chat").split("/api/")[0]
OLLAMA_MODEL = os.getenv("OLLAMA_MODEL", "qwen3-vl:4b-instruct")
OLLAMA_TIMEOUT = float(os.getenv("OLLAMA_TIMEOUT_SEC", "120"))
LLM_RETRIES = 1  # extra attempts after the first invalid response

# Cloud intent server (cloud/intent_server.py): POST /receive_intent
CLOUD_INTENT_URL = (
    f"http://{os.getenv('CLOUD_HOST', '127.0.0.1')}:{os.getenv('INTENT_PORT', '5000')}/receive_intent"
)
CLOUD_TIMEOUT = 5.0

FRAME_ID = "map"
DEFAULT_SAFETY_MARGIN_M = 0.45
# No manager speed default: omission delegates selection to the Edge VLM.
# Limits enforced by the Edge side (common/speed_policy.py, common/intent_contract.py)
MIN_SPEED_MPS = 0.01
MAX_SPEED_MPS = 0.35
MIN_SAFETY_MARGIN_M = 0.30
ALLOWED_ACTIONS = ("move",)
