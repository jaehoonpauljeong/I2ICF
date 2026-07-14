"""
Ollama (Llama3.1) → structured intent for LIMO.

Returns one of:
  action:    {mode:"action",    command:str,  speed:float, duration:float, target_class:None}
  detection: {mode:"detection", command:None, speed:None,  duration:None,  target_class:None}
  trace:     {mode:"trace",     command:None, speed:None,  duration:None,  target_class:str}
"""
import json
import re

import requests

from .config import OLLAMA_URL, OLLAMA_MODEL

_PROMPT = """\
You control a LIMO robot. Parse the user command and return ONLY a JSON object.

JSON fields:
- "mode": "action" | "detection" | "trace"
- "command": action name (action mode only, else null)
- "speed": float m/s (action mode only, else null)
- "duration": float seconds (action mode only, else null)
- "target_class": YOLO class string for trace mode (else null)

Action commands (use these exact strings):
  go straight | go straight slow | go straight fast
  move forward | go forward
  move back | move back slow | move backward | go back | stop
  rotate left | rotate right
  turn left | turn right | sharp left | sharp right
  curve left | curve right

Mode rules:
  "action"    → a single direct movement (go, move, turn, rotate, stop)
  "detection" → detect or greet people (keywords: detect, greet, find people, say hello, wave)
  "trace"     → follow or track a moving target (keywords: follow, track, chase, pursue, trace, keep up with)

Examples:
User: go forward
JSON: {"mode":"action","command":"go straight","speed":0.5,"duration":2.0,"target_class":null}

User: go forward for 3 seconds
JSON: {"mode":"action","command":"go straight","speed":0.5,"duration":3.0,"target_class":null}

User: move forward
JSON: {"mode":"action","command":"move forward","speed":0.5,"duration":2.0,"target_class":null}

User: move backward slowly
JSON: {"mode":"action","command":"move back slow","speed":0.2,"duration":2.0,"target_class":null}

User: go back
JSON: {"mode":"action","command":"move back","speed":0.5,"duration":2.0,"target_class":null}

User: turn right slowly
JSON: {"mode":"action","command":"turn right","speed":0.2,"duration":2.0,"target_class":null}

User: follow the person
JSON: {"mode":"trace","command":null,"speed":null,"duration":null,"target_class":"person"}

User: track the car in front
JSON: {"mode":"trace","command":null,"speed":null,"duration":null,"target_class":"car"}

User: chase the ball
JSON: {"mode":"trace","command":null,"speed":null,"duration":null,"target_class":"sports ball"}

User: greet people around you
JSON: {"mode":"detection","command":null,"speed":null,"duration":null,"target_class":null}

User: find and wave at someone
JSON: {"mode":"detection","command":null,"speed":null,"duration":null,"target_class":null}

Return ONLY valid JSON, no explanation.\
"""


def infer(text: str) -> dict:
    prompt = f"{_PROMPT}\n\nUser: {text}\nJSON:"
    try:
        resp = requests.post(
            f"{OLLAMA_URL}/api/generate",
            json={"model": OLLAMA_MODEL, "prompt": prompt, "stream": False, "format": "json"},
            timeout=30,
        )
        raw = resp.json().get("response", "")
        m = re.search(r'\{.*\}', raw, re.DOTALL)
        if m:
            return _normalize(json.loads(m.group()))
    except Exception as e:
        print(f"[LLM] Error: {e}")
    return _fallback(text)


def _normalize(d: dict) -> dict:
    mode = d.get("mode", "action")
    if mode not in ("action", "detection", "trace"):
        mode = "action"

    if mode == "action":
        return {
            "mode": "action",
            "command": d.get("command") or "stop",
            "speed": float(d["speed"]) if d.get("speed") is not None else 0.3,
            "duration": float(d["duration"]) if d.get("duration") is not None else 2.0,
            "target_class": None,
        }
    if mode == "trace":
        return {
            "mode": "trace",
            "command": None,
            "speed": None,
            "duration": None,
            "target_class": d.get("target_class") or "person",
        }
    # detection
    return {
        "mode": "detection",
        "command": None,
        "speed": None,
        "duration": None,
        "target_class": None,
    }


def _fallback(text: str) -> dict:
    t = text.lower()

    if any(w in t for w in ("follow", "track", "chase", "trace", "pursue")):
        return {"mode": "trace", "command": None, "speed": None, "duration": None, "target_class": "person"}
    if any(w in t for w in ("detect", "greet", "wave")):
        return {"mode": "detection", "command": None, "speed": None, "duration": None, "target_class": None}

    # Movement keyword matching
    if any(w in t for w in ("forward", "ahead", "straight", "front")):
        if "slow" in t:
            cmd = "go straight slow"
        elif any(w in t for w in ("fast", "quick", "speed")):
            cmd = "go straight fast"
        else:
            cmd = "go straight"
    elif any(w in t for w in ("back", "backward", "reverse")):
        cmd = "move back slow" if "slow" in t else "move back"
    elif "sharp left" in t:
        cmd = "sharp left"
    elif "sharp right" in t:
        cmd = "sharp right"
    elif "curve left" in t:
        cmd = "curve left"
    elif "curve right" in t:
        cmd = "curve right"
    elif any(w in t for w in ("rotate left", "spin left")):
        cmd = "rotate left"
    elif any(w in t for w in ("rotate right", "spin right")):
        cmd = "rotate right"
    elif "left" in t:
        cmd = "turn left"
    elif "right" in t:
        cmd = "turn right"
    else:
        cmd = "stop"

    # Extract duration if mentioned (e.g. "for 3 seconds")
    duration = 2.0
    m = re.search(r'(\d+(?:\.\d+)?)\s*(?:second|sec|s\b)', t)
    if m:
        duration = float(m.group(1))

    return {"mode": "action", "command": cmd, "speed": 0.5, "duration": duration, "target_class": None}
