from ultralytics import YOLO
import cv2

# 1. 모델 로드
model = YOLO("detector/yolov8s.pt")

# 2. 웹캠 열기
cap = cv2.VideoCapture(0)

# 4. 프레임 반복 처리
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # 5. YOLOv8 추론
    results = model.predict(source=frame, classes=None, conf=0.8)  # person class: 0, 14

    # 6. 결과 시각화
    annotated_frame = results[0].plot()

    # 7. 프레임 저장 및 시각화
    cv2.imshow("YOLOv8 Detection", annotated_frame)
    if cv2.waitKey(1) == ord("q"):
        break

# 8. 종료 처리
cap.release()
cv2.destroyAllWindows()
