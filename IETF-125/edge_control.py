import cv2
from ultralytics import YOLO
import random, time, datetime, requests, base64, threading, queue, math
from collections import deque
import yaml  # 정책 YAML 읽기용

# ---------------------------
# Setting
# ---------------------------
IMO_BASE   = "http://192.168.50.165"
CLOUD_BASE = "http://localhost"

STREAM_URL = f"{IMO_BASE}:8000/video"
ODOM_URL   = f"{IMO_BASE}:8000/odometry"
LIDAR_URL  = f"{IMO_BASE}:8000/lidar"
SERVER_URL = f"{CLOUD_BASE}:8080/inference"

# 거리 전송 API (기존 stop용)
DIST_API   = f"{IMO_BASE}:8001/control/distance"

# cmd_vel 제어 API (추가)
CMD_API = f"{IMO_BASE}:8001/control/cmd_vel"

# 정책 파일
POLICY_FILE = "received_policy.yaml"

INFER_HZ      = 10.0
SEND_INTERVAL = 1.0
JPEG_QUALITY  = 70
TIMEOUT_GET   = 0.3
TIMEOUT_POST  = 0.5

# Lidar Setting
LIDAR_HZ       = 10.0
LIDAR_TO       = 0.5
LIDAR_LEN      = 401
CAMERA_FOV_DEG = 71.0

# GPS 근사 변환 기준
BASE_LAT = 37.501000
BASE_LON = 127.036000

# ---------------------------
# 회피 시퀀스 설정
# ---------------------------
# 첫 stop 발생 기준 거리
STOP_TRIGGER = 0.7

# S-curve parameter
K_TIME = 2.0        # k초
TURN_ANG = 0.35     # n도 조향(rad/s)
STRAIGHT_ANG = 0.0

# 회피 중 전진 속도
FORWARD_SPEED = 0.2


def fake_gps_from_odom(x_m, y_m):
    return {
        "lat": BASE_LAT + x_m / 111000.0,
        "lon": BASE_LON + y_m /  88000.0,
    }


# ---------------------------
# 정책 체크 함수
# ---------------------------
def allow_send_distance():
    """
    YAML의 ingress-action을 읽어서 True/False 반환
    - pass -> True
    - drop -> False
    """
    try:
        with open(POLICY_FILE, "r") as f:
            data = yaml.safe_load(f)

        action = data["i2nsf-security-policy"]["rules"]["action"]["packet-action"]["ingress-action"]
        return action == "pass"

    except Exception as e:
        print(f"[Policy ERROR] YAML 파싱 실패 또는 필드 없음: {e}")
        return False


# ---------------------------
# 전역 공유 구조
# ---------------------------
frame_q   = deque(maxlen=1)
send_q    = queue.Queue(maxsize=10)

odom_lock = threading.Lock()
odom_cache= {"gps": {"lat": None, "lon": None}, "speed": 0.0}

lidar_lock  = threading.Lock()
lidar_cache = {
    "angle_min": None,
    "angle_increment": None,
    "ranges": []
}

stop_evt  = threading.Event()

# ---------------------------
# 회피 상태 머신
# ---------------------------
avoid_state = {
    "active": False,      # 회피 진행 중 여부
    "stage": 0,           # 1 -> +n turn, 2 -> -n turn, 3 -> +n turn, 4 -> straight
    "start_time": 0.0,    # 현재 단계 시작 시각
    "direction": 1        # 1 = 오른쪽 회피, -1 = 왼쪽 회피
}

# "첫 stop 발생" 감지용
last_stop = False


# ---------------------------
# Robocar(LIMO) 제어용 함수
# ---------------------------
_last_dist_sent = {"t": 0.0, "d": None}

def send_distance_to_robocar(dist_m: float, min_interval=0.2, delta=0.05):
    """
    dist_m: 최근접 사람 거리(m)
    min_interval: 최소 전송 간격(s)
    delta: 이전 값 대비 변화가 이 이상일 때만 전송
    정책이 pass일 때만 전송
    """
    now    = time.time()
    prev_t = _last_dist_sent["t"]
    prev_d = _last_dist_sent["d"]

    if dist_m is None:
        return

    # 정책 체크
    if not allow_send_distance():
        print(f"[Policy] ingress-action=drop -> distance 전송 차단 ({dist_m:.2f} m)")
        return

    # 전송 간격 제한
    if now - prev_t < min_interval:
        return

    # 변화량이 너무 작으면 생략
    if (prev_d is not None) and (abs(prev_d - dist_m) < delta):
        return

    try:
        r = requests.post(DIST_API, json={"distance": float(dist_m)}, timeout=0.3)
        if r.status_code == 200:
            _last_dist_sent["t"] = now
            _last_dist_sent["d"] = float(dist_m)
            print(f"[Robocar] distance -> {dist_m:.2f} m 전송 OK")
        else:
            print(f"[Robocar] distance 전송 실패: {r.status_code}")
    except Exception as e:
        print(f"[Robocar] distance 전송 오류: {e}")


def send_cmd(linear, angular):
    """
    imo_control.py 의 /control/cmd_vel 로 주행 명령 전송
    """
    try:
        requests.post(
            CMD_API,
            json={
                "linear_x": float(linear),
                "angular_z": float(angular)
            },
            timeout=0.2
        )
    except Exception as e:
        print("[CMD ERROR]", e)


# ---------------------------
# Thread 1. CaptureThread
# ---------------------------
def capture_loop():
    cap = cv2.VideoCapture(STREAM_URL)
    if not cap.isOpened():
        print("[Capture] 스트림 오픈 실패")
        stop_evt.set()
        return

    print("[Capture] 시작")
    while not stop_evt.is_set():
        ok, frame = cap.read()
        if ok:
            frame_q.append(frame)
        else:
            time.sleep(0.005)

    cap.release()
    print("[Capture] 종료")


# ---------------------------
# Thread 2. OdomThread
# ---------------------------
def odom_loop():
    print("[Odom] 시작")
    poll_period = 0.1

    while not stop_evt.is_set():
        t0 = time.time()
        try:
            r = requests.get(ODOM_URL, timeout=TIMEOUT_GET)
            if r.status_code == 200:
                odom = r.json()
                x = odom["pose"]["position"]["x"]
                y = odom["pose"]["position"]["y"]
                speed = odom["twist"]["linear"]["x"]
                gps = fake_gps_from_odom(x, y)

                with odom_lock:
                    odom_cache["gps"] = gps
                    odom_cache["speed"] = float(speed)
        except Exception:
            pass

        dt = time.time() - t0
        if dt < poll_period:
            time.sleep(poll_period - dt)

    print("[Odom] 종료")


# ---------------------------
# Thread 3. InferThread
# ---------------------------
def infer_loop(model):
    global last_stop

    print("[Infer] 시작")
    last_infer = 0.0
    last_send  = time.time()

    while not stop_evt.is_set():
        if not frame_q:
            time.sleep(0.005)
            continue

        now = time.time()
        if now - last_infer < (1.0 / INFER_HZ):
            time.sleep(0.001)
            continue

        frame = frame_q[-1]
        res = model.predict(source=frame, conf=0.3, verbose=False)[0]

        boxes = res.boxes
        detected = []

        for box in boxes:
            cls_id = int(box.cls[0])
            class_name = res.names[cls_id]
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = float(box.conf[0])

            detected.append({
                "class": class_name,
                "bbox": [x1, y1, x2, y2],
                "conf": round(conf, 2),
                "cls_id": cls_id
            })

            color = class_colors.get(cls_id, (0, 255, 0))
            label = f"{class_name} {round(conf, 2)}"
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # --- LiDAR snapshot & closest person ---
        with lidar_lock:
            angle_min = lidar_cache["angle_min"]
            angle_inc = lidar_cache["angle_increment"]
            ranges    = list(lidar_cache["ranges"])

        closest_person = None

        if angle_min is not None and angle_inc is not None and len(ranges) > 0 and len(detected) > 0:
            image_w = frame.shape[1]
            fov_rad = math.radians(CAMERA_FOV_DEG)
            fov_half = fov_rad / 2.0
            best = (float("inf"), None)

            for obj in detected:
                # 사람만 대상으로 회피 판단
                if obj["class"] != "person":
                    continue

                x1, y1, x2, y2 = obj["bbox"]
                cx = (x1 + x2) // 2

                ratio = cx / image_w
                angle_rel = (ratio - 0.5) * fov_rad

                if abs(angle_rel) > fov_half:
                    continue

                angle_global = angle_rel
                idx = int((angle_global - angle_min) / angle_inc)

                if 0 <= idx < len(ranges):
                    dist = ranges[idx]

                    if 0.02 < dist < 12.0:
                        if dist < best[0]:
                            best = (dist, {
                                "class": obj["class"],
                                "conf": obj["conf"],
                                "bbox": obj["bbox"],
                                "distance": round(dist, 3),
                                "angle": round(math.degrees(angle_global), 2),
                                "center_x": cx,
                                "center_y": (y1 + y2) // 2
                            })

            if best[1] is not None:
                closest_person = best[1]

        # ------------------------------------
        # 회피 중이 아닐 때만 distance 전송
        # 회피 중에 계속 distance를 보내면
        # imo_control.py 쪽 e_stop이 유지되어
        # cmd_vel 회피 명령이 막힐 수 있음
        # ------------------------------------
        if not avoid_state["active"]:
            dist_to_send = closest_person["distance"] if closest_person else None
            send_distance_to_robocar(dist_to_send)

        # ------------------------------------
        # 현재 stop 상태 판단
        # ------------------------------------
        current_stop = False
        if closest_person and closest_person["distance"] <= STOP_TRIGGER:
            current_stop = True

        # ------------------------------------
        # 첫 stop 발생 시에만 회피 시작
        # False -> True 로 바뀌는 순간만 트리거
        # ------------------------------------
        if (not last_stop) and current_stop and (not avoid_state["active"]):
            avoid_state["active"] = True
            avoid_state["stage"] = 1
            avoid_state["start_time"] = time.time()

            # 사람이 왼쪽에 있으면 오른쪽으로 회피
            # 사람이 오른쪽에 있으면 왼쪽으로 회피
            if closest_person["center_x"] < frame.shape[1] / 2:
                avoid_state["direction"] = 1
            else:
                avoid_state["direction"] = -1

            print("[AVOID] 시작")

        # 상태 업데이트
        last_stop = current_stop

        # ------------------------------------
        # S-curve 회피 시퀀스
        # 1) +n 조향 k초
        # 2) -n 조향 2k초
        # 3) +n 조향 k초
        # 4) 바퀴 정렬 직진
        # ------------------------------------

        if avoid_state["active"]:
            elapsed = time.time() - avoid_state["start_time"]
            stage = avoid_state["stage"]
            direction = avoid_state["direction"]

            # 디버그 표시
            cv2.putText(frame, f"AVOID STAGE: {stage}", (20, 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(frame, f"DIR: {direction}", (20, 65),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            if stage == 1:
                # 오른쪽 n도 (direction 적용)
                send_cmd(FORWARD_SPEED, TURN_ANG * direction)

                if elapsed > K_TIME:
                    avoid_state["stage"] = 2
                    avoid_state["start_time"] = time.time()


            elif stage == 2:
                # 왼쪽 -n도 (2k초)
                send_cmd(FORWARD_SPEED, -TURN_ANG * direction)

                if elapsed > (2 * K_TIME):
                    avoid_state["stage"] = 3
                    avoid_state["start_time"] = time.time()


            elif stage == 3:
                # 다시 오른쪽 n도
                send_cmd(FORWARD_SPEED, TURN_ANG * direction)

                if elapsed > K_TIME:
                    avoid_state["stage"] = 4
                    avoid_state["start_time"] = time.time()


            elif stage == 4:
                # 바퀴 정렬 후 직진
                send_cmd(FORWARD_SPEED, STRAIGHT_ANG)

                if elapsed > K_TIME:
                    avoid_state["active"] = False
                    avoid_state["stage"] = 0
                    avoid_state["start_time"] = 0.0

                    send_cmd(0.0, 0.0)

                    print("[AVOID] 종료")

        # ------------------------------------
        # 1초마다 서버 전송
        # ------------------------------------
        if now - last_send >= SEND_INTERVAL:
            timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            ok, buf = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
            img_b64 = base64.b64encode(buf).decode('utf-8') if ok else None

            with odom_lock:
                gps   = dict(odom_cache["gps"])
                speed = float(odom_cache["speed"])

            payload = {
                "timestamp": timestamp,
                "gps": gps,
                "robocar_speed": speed,
                "objects": [
                    {"class": o["class"], "conf": o["conf"], "bbox": o["bbox"]}
                    for o in detected
                ],
                "image": img_b64,
                "lidar_available": angle_min is not None and angle_inc is not None and len(ranges) > 0,
                "closest_person": closest_person,
                "avoid_active": avoid_state["active"],
                "avoid_stage": avoid_state["stage"]
            }

            try:
                if send_q.full():
                    send_q.get_nowait()
                send_q.put_nowait(payload)
            except queue.Full:
                pass

            last_send = now

        last_infer = now

    print("[Infer] 종료")


# ---------------------------
# Thread 4. SenderThread
# ---------------------------
def sender_loop():
    print("[Sender] 시작")
    while not stop_evt.is_set():
        try:
            data = send_q.get(timeout=0.2)
        except queue.Empty:
            continue

        try:
            r = requests.post(SERVER_URL, json=data, timeout=TIMEOUT_POST)
            if r.status_code != 200:
                print(f"[Sender] 전송 실패: {r.status_code}")
            else:
                print(f"[Sender] 전송 성공: {data.get('timestamp')}")
        except Exception as e:
            print(f"[Sender] 오류: {e}")
        finally:
            send_q.task_done()

    print("[Sender] 종료")


# ---------------------------
# Thread 5. LidarThread
# ---------------------------
def lidar_loop():
    print("[LiDAR] 시작")
    period = 1.0 / LIDAR_HZ

    while not stop_evt.is_set():
        t0 = time.time()
        try:
            r = requests.get(LIDAR_URL, timeout=LIDAR_TO)
            if r.status_code == 200:
                data = r.json()
                angle_min = data.get("angle_min")
                angle_inc = data.get("angle_increment")
                ranges    = data.get("ranges", [])

                if len(ranges) < LIDAR_LEN:
                    ranges = ranges + [0.0] * (LIDAR_LEN - len(ranges))
                elif len(ranges) > LIDAR_LEN:
                    ranges = ranges[:LIDAR_LEN]

                with lidar_lock:
                    lidar_cache["angle_min"]       = angle_min
                    lidar_cache["angle_increment"] = angle_inc
                    lidar_cache["ranges"]          = ranges
        except Exception:
            pass

        dt = time.time() - t0
        if dt < period:
            time.sleep(period - dt)

    print("[LiDAR] 종료")


# ---------------------------
# Main
# ---------------------------
if __name__ == "__main__":
    model = YOLO("detector/yolov8s.pt")
    class_names = model.names

    random.seed(42)
    class_colors = {
        cls_id: tuple(random.randint(0, 255) for _ in range(3))
        for cls_id in class_names
    }

    th_cap   = threading.Thread(target=capture_loop, daemon=True)
    th_odom  = threading.Thread(target=odom_loop,   daemon=True)
    th_lidar = threading.Thread(target=lidar_loop,  daemon=True)
    th_inf   = threading.Thread(target=infer_loop,  args=(model,), daemon=True)
    th_send  = threading.Thread(target=sender_loop, daemon=True)

    th_cap.start()
    th_odom.start()
    th_lidar.start()
    th_inf.start()
    th_send.start()

    try:
        while not stop_evt.is_set():
            if frame_q:
                cv2.imshow("YOLO Detection (Live)", frame_q[-1])

            if cv2.waitKey(1) & 0xFF == ord('q'):
                stop_evt.set()
                break

            time.sleep(0.005)

    finally:
        stop_evt.set()
        th_cap.join(timeout=1.0)
        th_odom.join(timeout=1.0)
        th_lidar.join(timeout=1.0)
        th_inf.join(timeout=1.0)
        th_send.join(timeout=1.0)
        cv2.destroyAllWindows()