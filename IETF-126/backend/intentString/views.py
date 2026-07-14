"""
Django Front API views.

Pipeline:
  POST /api/infer/  →  Llama  →  Function Table  →  Sequence Parser  →  RosBridge  →  LIMO
  POST /api/stop/   →  stop all sessions + stop robot
  GET  /api/status/ →  current session status
  GET  /api/logs/   →  recent intent + action logs
"""
import json
import time

from rest_framework.response import Response
from rest_framework.views import APIView
from rest_framework import status as drf_status

from .models import IntentLog, ActionLog
from .services import limo_llm, function_table, sequence_parser
from .services.rosbridge import bridge
from .services.yolo_detection import detection_session
from .services.yolo_trace import trace_session


def _stop_all() -> None:
    """Stop all active YOLO sessions. Robot stop is handled by each command path."""
    if detection_session.running:
        detection_session.stop()
    if trace_session.running:
        trace_session.stop()


class InferView(APIView):
    """
    Main pipeline endpoint.
    Body: {"text": "<natural language command>"}
    """
    def post(self, request):
        text = request.data.get("text", "").strip()
        if not text:
            return Response({"error": "text field required"}, status=drf_status.HTTP_400_BAD_REQUEST)

        # ── 1. Llama inference ────────────────────────────────────────────
        intent = limo_llm.infer(text)
        mode = intent["mode"]

        print(f"[Infer] text='{text}' → intent={intent}")

        # Log intent
        IntentLog.objects.create(input_text=text, intent_json=json.dumps(intent))

        _stop_all()  # always stop previous session first

        # ── 2. Function Table routing ─────────────────────────────────────

        if mode == "detection":
            detection_session.start()
            return Response({
                "input": text,
                "mode": "detection",
                "status": "detection+greet loop started",
            })

        if mode == "trace":
            target = intent["target_class"] or "person"
            trace_session.start(target_class=target)
            return Response({
                "input": text,
                "mode": "trace",
                "target_class": target,
                "status": "trace loop started",
            })

        # ── 3. Action: Function Table → Sequence Parser → RosBridge ──────
        command = intent["command"]
        speed = intent["speed"]
        duration = intent["duration"]

        linear_x, angular_z = function_table.resolve(command, speed)
        step = sequence_parser.build_step(command, linear_x, angular_z, duration)
        results = sequence_parser.execute([step])
        print(f"[Infer] step={step}  results={results}")

        # Log action
        ActionLog.objects.create(
            intent_json=json.dumps(intent),
            steps_json=json.dumps([step]),
            results_json=json.dumps(results),
        )

        return Response({
            "input": text,
            "mode": "action",
            "intent": intent,
            "step": step,
            "results": results,
        })


class StopView(APIView):
    """Stop all active sessions and halt robot."""
    def post(self, request):
        _stop_all()
        bridge.stop_robot()
        return Response({"status": "stopped"})


class StatusView(APIView):
    """Return current session status."""
    def get(self, request):
        return Response({
            "rosbridge_connected": bridge.connected,
            "detection_running": detection_session.running,
            "trace_running": trace_session.running,
        })


class LogView(APIView):
    """Return 20 most recent logs."""
    def get(self, request):
        intents = [
            {"id": l.id, "input": l.input_text, "intent": json.loads(l.intent_json), "at": l.created_at.isoformat()}
            for l in IntentLog.objects.all()[:20]
        ]
        actions = [
            {"id": l.id, "results": json.loads(l.results_json), "at": l.created_at.isoformat()}
            for l in ActionLog.objects.all()[:20]
        ]
        return Response({"intents": intents, "actions": actions})
