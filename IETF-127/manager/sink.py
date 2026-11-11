"""
Where a finished intent goes. A sink is any object with deliver(result, meta).
LocalFileSink stores it on disk; HttpSink posts it to the Cloud intent server.
Other transports (e.g. A2A) can be added the same way without touching intent_llm.py.
"""
import json
import re
from datetime import datetime
from pathlib import Path

import requests

from . import config

OUTPUT_DIR = Path(__file__).resolve().parent / "outputs"


class LocalFileSink:
    """
    outputs/<timestamp>_<intent_id>.json   the policy JSON itself ({"intent": ...})
    outputs/log.jsonl                      one line per request: input, source, corrections, file
    """

    def __init__(self, directory: Path = OUTPUT_DIR):
        self.dir = Path(directory)
        self.dir.mkdir(parents=True, exist_ok=True)

    def next_seq(self) -> int:
        """Continue numbering across runs (move_001, move_002, ...)."""
        nums = [
            int(m.group(1))
            for p in self.dir.glob("*_move_*.json")
            if (m := re.search(r"_move_(\d+)\.json$", p.name))
        ]
        return max(nums, default=0) + 1

    def deliver(self, result: dict, meta: dict, input_text: str = "") -> Path:
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        path = self.dir / f"{stamp}_{result['intent']['intent_id']}.json"
        path.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        entry = {
            "time": datetime.now().isoformat(timespec="seconds"),
            "input": input_text,
            "file": path.name,
            "source": meta["source"],
            "corrections": meta["corrections"],
            "error": meta["error"],
        }
        with open(self.dir / "log.jsonl", "a", encoding="utf-8") as f:
            f.write(json.dumps(entry, ensure_ascii=False) + "\n")
        return path

    def record_send(self, filename: str, outcome: dict) -> None:
        """Append the delivery outcome for an already-saved file to log.jsonl."""
        entry = {
            "time": datetime.now().isoformat(timespec="seconds"),
            "file": filename,
            "sent": outcome,
        }
        with open(self.dir / "log.jsonl", "a", encoding="utf-8") as f:
            f.write(json.dumps(entry, ensure_ascii=False) + "\n")


class HttpSink:
    """
    POST the intent to the Cloud intent server (cloud/intent_server.py, /receive_intent).
    The server expects the intent object itself, not wrapped in {"intent": ...}.
    Never raises: returns {"ok": bool, "status": int|None, "response"|"error": ...}.
    """

    def __init__(self, url: str = None, timeout: float = None):
        self.url = url or config.CLOUD_INTENT_URL
        self.timeout = timeout or config.CLOUD_TIMEOUT

    def deliver(self, result: dict, meta: dict = None, input_text: str = "") -> dict:
        try:
            resp = requests.post(self.url, json=result["intent"], timeout=self.timeout)
        except requests.RequestException as e:
            return {"ok": False, "status": None, "url": self.url, "error": str(e)}
        try:
            body = resp.json()
        except ValueError:
            body = resp.text[:500]
        return {"ok": resp.ok, "status": resp.status_code, "url": self.url, "response": body}
