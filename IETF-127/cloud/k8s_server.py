"""Optional Cloud report query gateway. JSON maintenance files belong to Edge only."""
import requests
from flask import Flask, jsonify, request
from common.config import NET_CFG, EDGE_BASE_URL, BIND_HOST
app = Flask(__name__)

@app.get("/reports")
def reports():
    try:
        response = requests.get(EDGE_BASE_URL + "/reports", params=request.args, timeout=3.)
        return jsonify(response.json()), response.status_code
    except (requests.RequestException, ValueError) as exc:
        return jsonify({"ok":False,"error":"EDGE_UNAVAILABLE","message":str(exc)}),503

@app.post("/inference")
def obsolete_incremental_log():
    return jsonify({"ok":False,"error":"INCREMENTAL_LOG_DISABLED","message":"Use Edge final scenario logs and /reports"}),410

if __name__ == "__main__":
    app.run(host=BIND_HOST, port=NET_CFG.k8s_port, threaded=True)
