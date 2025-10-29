from flask import Flask, request, jsonify
import datetime
import os
import json
import base64

# --- Flask 앱 초기화 ---
app = Flask(__name__)

# --- 저장 디렉토리 설정 ---
LOG_DIR = "inference_logs"
IMAGE_DIR = os.path.join(LOG_DIR, "images")
os.makedirs(LOG_DIR, exist_ok=True)
os.makedirs(IMAGE_DIR, exist_ok=True)

@app.route('/inference', methods=['POST'])
def receive_inference():
    try:
        # --- 클라이언트로부터 JSON 데이터 수신 ---
        data = request.get_json(force=True)

        # --- timestamp 설정 (없으면 현재 시각 사용) ---
        timestamp = data.get("timestamp", datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))
        safe_timestamp = timestamp.replace(":", "_")  # 윈도우 등 호환을 위해 콜론 제거

        # --- JSON 로그 파일 저장 ---
        json_path = os.path.join(LOG_DIR, f"{safe_timestamp}.json")
        with open(json_path, "w") as f:
            json.dump(data, f, indent=2)

        # --- 이미지 데이터가 포함된 경우 저장 ---
        if "image" in data:
            try:
                img_b64 = data["image"]
                img_bytes = base64.b64decode(img_b64)
                img_path = os.path.join(IMAGE_DIR, f"{safe_timestamp}.jpg")
                with open(img_path, "wb") as f:
                    f.write(img_bytes)
                print(f"[{timestamp}] 이미지 저장 완료 → {img_path}")
            except Exception as e:
                print(f"[{timestamp}] 이미지 디코딩 실패: {e}")
        else:
            print(f"[{timestamp}] 이미지 없음")

        # --- 디버그 출력 ---
        print(f"[{timestamp}] 수신 완료 | 객체 수: {len(data.get('objects', []))}")
        return jsonify({"status": "ok"}), 200

    except Exception as e:
        print(f"[에러 발생] {e}")
        return jsonify({"status": "error", "message": str(e)}), 400

# --- 서버 실행 ---
if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8080)
