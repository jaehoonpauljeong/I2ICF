"""Cloud layer: translate/validate user intents and forward only to Edge."""
import copy
import os
import yaml
from pathlib import Path
import threading
import time
import requests
from flask import Flask, jsonify, request
from common.config import NET_CFG, EDGE_BASE_URL, BIND_HOST
from common.intent_contract import normalize_user_intent

app = Flask(__name__)
intent_lock = threading.RLock()
active_intent_state = {"has_active_intent":False,"intent":None,"received_at":None,"version":time.time_ns()}

@app.post("/receive_intent")
def receive_intent():
    try:
        raw = request.get_json(force=True)
        if not isinstance(raw, dict):
            raise ValueError("Intent must be an object")
        if str(raw.get("action", "")).lower().strip() == "report":
            # Queries never replace the active movement intent or fence the robot.
            response = requests.get(EDGE_BASE_URL + "/reports", params={
                k:v for k,v in {"intent_id":raw.get("for_intent_id"),
                               "intent_version":raw.get("for_intent_version")}.items() if v is not None}, timeout=3.)
            return jsonify(response.json()), response.status_code
        normalized = normalize_user_intent(raw)
        with intent_lock:
            version = max(time.time_ns(), active_intent_state["version"] + 1)
            response = requests.post(EDGE_BASE_URL + "/intents", json={"intent":normalized,"version":version}, timeout=5.)
            result = response.json()
            if response.status_code == 200 and result.get("ok"):
                active_intent_state.update(has_active_intent=True,intent=normalized,received_at=time.time(),version=version)
            return jsonify({**result,"intent":normalized,"version":version}), response.status_code
    except requests.RequestException as exc:
        return jsonify({"ok":False,"error":"EDGE_UNAVAILABLE","message":str(exc)}),503
    except (ValueError, TypeError, KeyError) as exc:
        return jsonify({"ok":False,"error":"INVALID_INTENT","message":str(exc)}),400

@app.get("/current_intent")
def current_intent():
    with intent_lock:
        return jsonify(copy.deepcopy(active_intent_state))

@app.get("/reports")
def reports():
    try:
        response = requests.get(EDGE_BASE_URL + "/reports", params=request.args, timeout=3.)
        return jsonify(response.json()), response.status_code
    except (requests.RequestException, ValueError) as exc:
        return jsonify({"ok":False,"error":"EDGE_UNAVAILABLE","message":str(exc)}),503

@app.post("/receive_policy")
def receive_policy():
    try:
        policy = yaml.safe_load(request.data.decode("utf-8"))
        Path(os.getenv("POLICY_FILE", "received_policy.yaml")).write_text(yaml.safe_dump(policy,allow_unicode=True),encoding="utf-8")
        return jsonify({"status":"ok"})
    except (ValueError, OSError, yaml.YAMLError) as exc:
        return jsonify({"status":"error","message":str(exc)}),400

if __name__ == "__main__":
    app.run(host=BIND_HOST, port=NET_CFG.intent_port, threaded=True)
