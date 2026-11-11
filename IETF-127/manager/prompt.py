import json

_SCHEMA = """\
You convert a robot command into ONE JSON object and return ONLY that JSON.

Schema:
{"intent": {
  "intent_id": "move_001",
  "action": "move",
  "target_location": {"x": <number>, "y": <number>, "frame_id": "map"},
  "constraints": {"safety_margin_m": <number>, "speed_mps": <number>},
  "report": <true|false>
}}

Rules:
- action is always "move".
- x and y are copied EXACTLY from the user's text. Never round, convert or invent them.
- frame_id is always "map".
- safety_margin_m is 0.45 unless the user states another value.
- speed_mps in the schema is OPTIONAL. Include it only when the user explicitly states a numeric speed.
- If no numeric speed is stated, omit speed_mps entirely (not null and not a default).
- The Edge driving VLM selects an omitted speed from observations; the manager must not select it.
- report is true unless the user says not to report.
- Output JSON only, no explanation."""

# (user text, intent fields) — numbers here are what the model should copy
_EXAMPLES = [
    ("x 50, y 50 위치로 이동해줘", 50.0, 50.0, 0.45, None, True),
    ("x:30 y:40 좌표까지 속도 0.3으로 이동 후 보고해줘", 30.0, 40.0, 0.45, 0.3, True),
    ("go to x -16 y 0", -16.0, 0.0, 0.45, None, True),
    ("x는 3.5, y는 -2.25로 가", 3.5, -2.25, 0.45, None, True),
    ("y 7, x 12 로 이동", 12.0, 7.0, 0.45, None, True),
    ("move to (1, 2) at 0.3 m/s", 1.0, 2.0, 0.45, 0.3, True),
    ("x 10 y 20 으로 가는데 안전 여유 0.6", 10.0, 20.0, 0.6, None, True),
    ("x 4, y 5 이동하고 보고는 하지 마", 4.0, 5.0, 0.45, None, False),
]


def _intent_json(i, x, y, margin, speed, report):
    constraints = {"safety_margin_m": margin}
    if speed is not None:
        constraints["speed_mps"] = speed
    return json.dumps(
        {
            "intent": {
                "intent_id": f"move_{i:03d}",
                "action": "move",
                "target_location": {"x": x, "y": y, "frame_id": "map"},
                "constraints": constraints,
                "report": report,
            }
        },
        ensure_ascii=False,
    )


def build(text: str) -> str:
    shots = "\n\n".join(
        f"User: {t}\nJSON: {_intent_json(n, *rest)}"
        for n, (t, *rest) in enumerate(_EXAMPLES, start=1)
    )
    return f"{_SCHEMA}\n\nExamples:\n\n{shots}\n\nUser: {text}\nJSON:"
