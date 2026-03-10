from flask import Flask, request, jsonify
import yaml

app = Flask(__name__)

@app.route("/receive_policy", methods=["POST"])
def receive_policy():
    raw_yaml = request.data.decode("utf-8")

    print("\n=== 📥 Received YAML ===")
    print(raw_yaml)

    try:
        parsed = yaml.safe_load(raw_yaml)
        print("\n=== Parsed YAML ===")
        print(parsed)

        # 저장
        with open("/received_policy.yaml", "w", encoding="utf-8") as f:
            yaml.dump(parsed, f, allow_unicode=True, sort_keys=False)

        return jsonify({"status": "ok"}), 200
    except Exception as e:
        return jsonify({"status": "error", "msg": str(e)})

app.run(host="0.0.0.0", port=5000)