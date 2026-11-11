"""
Deterministic input parsing. The LLM never gets the final say on numbers:
x / y (and speed / margin when stated) are read from the user text here and
enforced on the LLM output in schema.py.
"""
import re

from .config import MAX_SPEED_MPS, MIN_SAFETY_MARGIN_M, MIN_SPEED_MPS

_NUM = r"[-+]?\d+(?:\.\d+)?"
_SEP = r"\s*(?:좌표)?\s*(?:는|은|=|:)?\s*"

_X_RE = re.compile(rf"(?<![A-Za-z])x{_SEP}({_NUM})", re.I)
_Y_RE = re.compile(rf"(?<![A-Za-z])y{_SEP}({_NUM})", re.I)
_PAIR_RE = re.compile(rf"\(\s*({_NUM})\s*,\s*({_NUM})\s*\)")
_SPEED_RE = re.compile(
    rf"(?:({_NUM})\s*(?:m/s|mps|미터\s*/\s*초)|(?:속도|speed)\s*(?:는|은|=|:)?\s*({_NUM}))",
    re.I,
)
_MARGIN_RE = re.compile(
    rf"(?:safety\s*margin|margin|안전\s*(?:거리|여유|마진))\s*(?:은|는|=|:)?\s*({_NUM})",
    re.I,
)


class InputError(ValueError):
    """The user text cannot be turned into a valid move intent."""


def _normalize(text: str) -> str:
    return text.replace("−", "-").replace("–", "-")


def _one(pattern, text, name):
    values = {float(m.group(1)) for m in pattern.finditer(text)}
    if len(values) > 1:
        raise InputError(f"ambiguous {name}: {sorted(values)}")
    return values.pop() if values else None


def extract(text: str) -> dict:
    """Return {"x", "y", "speed_mps"?, "safety_margin_m"?}. Raises InputError."""
    text = _normalize(text)

    x = _one(_X_RE, text, "x")
    y = _one(_Y_RE, text, "y")
    if x is None and y is None:
        pairs = {(float(a), float(b)) for a, b in _PAIR_RE.findall(text)}
        if len(pairs) > 1:
            raise InputError(f"ambiguous coordinates: {sorted(pairs)}")
        if pairs:
            x, y = pairs.pop()
    if x is None or y is None:
        raise InputError("need both x and y, e.g. 'go to x 50, y 50'")

    out = {"x": x, "y": y}

    speeds = {float(a or b) for a, b in _SPEED_RE.findall(text)}
    if len(speeds) > 1:
        raise InputError(f"ambiguous speed: {sorted(speeds)}")
    speed = speeds.pop() if speeds else None
    if speed is not None:
        if not MIN_SPEED_MPS <= speed <= MAX_SPEED_MPS:
            raise InputError(f"speed must be in [{MIN_SPEED_MPS}, {MAX_SPEED_MPS}] m/s, got {speed}")
        out["speed_mps"] = speed

    margin = _one(_MARGIN_RE, text, "safety margin")
    if margin is not None:
        if margin < MIN_SAFETY_MARGIN_M:
            raise InputError(f"safety margin must be >= {MIN_SAFETY_MARGIN_M} m, got {margin}")
        out["safety_margin_m"] = margin

    return out
