import cv2
from ultralytics import YOLO
import random
import time
import datetime
import requests

# --- 1. 모델 로드 ---
model = YOLO("detector/yolov8s.pt")
class_names = model.names

random.seed(42)
class_colors = {
    cls_id: tuple(random.randint(0, 255) for _ in range(3))
    for cls_id in class_names
}
# --- 2. 로보카 Flask 서버에서 영상 스트리밍 수신 ---
stream_url = "http://192.168.50.165:8000/video"
cap = cv2.VideoCapture(stream_url)

# --- 3. 루프: 프레임 수신 + YOLO 추론 ---
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print("스트리밍 수신 실패")
        continue

    # --- 4. YOLO 추론 수행 ---
    results = model.predict(source=frame, classes=None, conf=0.3, verbose=False)
    result = results[0]

    boxes = result.boxes
    detected = []

    for box in boxes:
        cls_id = int(box.cls[0])
        class_name = result.names[cls_id]
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        conf = float(box.conf[0])
        detected.append({
            "class": class_name,
            "bbox": [x1, y1, x2, y2],
            "conf": round(conf, 2),
            "cls_id": cls_id
        })

    # --- 5. 결과 출력 및 시각화 ---
    if detected:
        print(f"[{time.strftime('%H:%M:%S')}] 탐지 결과:")
        for obj in detected:
            print(f" - {obj['class']} ({obj['conf']}) → {obj['bbox']}")

        for obj in detected:
            x1, y1, x2, y2 = obj["bbox"]
            cls_id = obj["cls_id"]
            label = f"{obj['class']} {obj['conf']}"
            color = class_colors.get(cls_id, (0, 255, 0))  # fallback to green
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            # --- 6. 결과를 Flask 서버로 전송 ---
            timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")  # windows에선 콜론 인식 X
            robocar_speed = 1.0  # 임의 값 또는 추후 실제 센서 연동

            # 전송할 JSON 구조 정의
            payload = {
                "timestamp": timestamp,
                "robocar_speed": robocar_speed,
                "objects": [
                    {
                        "class": obj["class"],
                        "conf": obj["conf"],
                        "bbox": obj["bbox"]
                    } for obj in detected
                ]
            }

            try:
                res = requests.post("http://192.168.50.202:8080/inference", json=payload)
                if res.status_code != 200:
                    print(f"전송 실패: {res.status_code}")
            except Exception as e:
                print(f"POST 전송 에러: {e}")


    cv2.imshow("YOLO Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()