"""
python3 -m manager.run "x:30 y:40 좌표까지 속도 0.2로 이동 후 보고해줘" [--send]

Always saves the intent under manager/outputs/. With --send it is also posted to
the Cloud intent server (CLOUD_HOST / INTENT_PORT), which moves the robot.
"""
import argparse
import json
import sys

from . import intent_llm
from .extract import InputError
from .sink import HttpSink, LocalFileSink


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("text", nargs="+", help="natural-language command")
    ap.add_argument("--send", action="store_true", help="POST the intent to the Cloud intent server")
    args = ap.parse_args(argv)
    text = " ".join(args.text).strip()

    local = LocalFileSink()
    intent_llm.set_next_seq(local.next_seq())
    try:
        result, meta = intent_llm.infer(text)
    except InputError as e:
        print(f"input error: {e}", file=sys.stderr)
        return 2

    path = local.deliver(result, meta, input_text=text)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    print(f"[source={meta['source']}] corrections={meta['corrections']} error={meta['error']}", file=sys.stderr)
    print(f"saved: {path}", file=sys.stderr)

    if args.send:
        outcome = HttpSink().deliver(result)
        local.record_send(path.name, outcome)
        print(f"sent: {json.dumps(outcome, ensure_ascii=False)}", file=sys.stderr)
        return 0 if outcome["ok"] else 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
