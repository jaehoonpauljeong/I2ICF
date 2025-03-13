from flask import Flask, request, jsonify
import requests

# Flask 애플리케이션 생성
app = Flask(__name__)

# Analyzer IP & SDVUser IP 설정
#Analyzer_IP = '10.152.183.194'
SDVUSER_IP = '10.152.183.240'
SDVUSER_PORT = 5000

# 메시지 파일 저장 경로
MESSAGE_FILE = '/app/SDV_UE_messages.txt'

# 메시지 포워딩 함수
def forward_message(message):
    try:
        response = requests.post(f'http://{SDVUSER_IP}:{SDVUSER_PORT}/receive_message', data=message)
        print(f"Forwarded to SDVUser({SDVUSER_IP}): {response.status_code}, {response.text}")
    except Exception as e:
        print(f"Failed to forward message: {e}")

# 메시지 수신 핸들러 (포트 5000)
@app.route('/receive_message', methods=['POST'])
def receive_message():
    message = request.data.decode('utf-8')
    print("Analyzer received message from SDV1:")
    print(message)

    # 파일에 메시지 저장
    with open(MESSAGE_FILE, 'a') as file:
        file.write(f"Port 5000: {message}\n")

    # 메시지 포워딩
    forward_message(message)

    return jsonify({"message": "Message received and forwarded from port 5000"}), 200

# Flask 애플리케이션 실행
if __name__ == '__main__':
    app.run(host="0.0.0.0", port=5000, debug=True)
