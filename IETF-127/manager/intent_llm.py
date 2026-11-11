"""
Natural language -> intent JSON (few-shot, Ollama).
Counterpart of IETF-126 limo_llm.py: infer(text) is the single entry point.
"""
import itertools
import json
import re

import requests

from . import config, prompt
from .extract import InputError, extract
from .schema import SchemaError, check_shape, finalize, from_input_only

_seq = itertools.count(1)


def set_next_seq(n: int) -> None:
    global _seq
    _seq = itertools.count(n)


def ollama_generate(text_prompt: str) -> str:
    resp = requests.post(
        f"{config.OLLAMA_BASE_URL}/api/generate",
        json={
            "model": config.OLLAMA_MODEL,
            "prompt": text_prompt,
            "stream": False,
            "format": "json",
            "options": {"temperature": 0},
        },
        timeout=config.OLLAMA_TIMEOUT,
    )
    resp.raise_for_status()
    return resp.json().get("response", "")


def _parse(raw: str):
    m = re.search(r"\{.*\}", raw, re.DOTALL)
    if not m:
        raise SchemaError("no JSON object in response")
    try:
        return json.loads(m.group())
    except json.JSONDecodeError as e:
        raise SchemaError(f"invalid JSON: {e}")


def infer(text: str, llm=ollama_generate):
    """
    Returns (result, meta).
      result: {"intent": {...}}  — the policy JSON
      meta:   {"source": "llm"|"rules", "corrections": [...], "error": str|None}
    Raises InputError when the text has no usable x/y (nothing is sent to the LLM).
    """
    extracted = extract(text)  # raises InputError
    seq = next(_seq)
    text_prompt = prompt.build(text)

    last_error = None
    for _ in range(1 + config.LLM_RETRIES):
        try:
            intent = check_shape(_parse(llm(text_prompt)))
            result, corrections = finalize(intent, extracted, seq)
            return result, {"source": "llm", "corrections": corrections, "error": None}
        except (SchemaError, requests.RequestException) as e:
            last_error = str(e)

    return from_input_only(extracted, seq), {
        "source": "rules",
        "corrections": [],
        "error": last_error,
    }
