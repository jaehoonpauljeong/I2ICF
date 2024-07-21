import requests
import random
from flask import Flask, request, Response
import pytz
from datetime import datetime

app = Flask(__name__)

@app.route('/receive_message', methods=['POST'])
def receive_message():

    # 캐나다 밴쿠버 시간대 설정
    vancouver_tz = pytz.timezone('America/Vancouver')

    # 현재 시간 가져오기
    current_time = datetime.now(vancouver_tz).strftime('%Y-%m-%d %H:%M:%S')


    xml_data = request.data.decode('utf-8')
    print("Received XML from computer 4:")
    print(xml_data)

    # "speed" 값을 출력
    speed = random.randint(0, 100)
    message = f"SDV1 speed is {speed}km/h at {current_time}"
    print(message)
    print()

    # 컴퓨터 5로 speed 메시지 전송
    url = "http://10.60.0.4:9091/receive_message"
    try:
        response = requests.post(url, data=message, headers={'Content-Type': 'text/plain'}, timeout=5)  # 타임아웃 설정
        print(f"Sent '{message}' to computer 5: {response.status_code}")
        print("Response from computer 5:")
        print(response.text)
    except requests.exceptions.RequestException as e:
        print(f"Failed to send message to computer 5: {e}")

    return Response("Message received", status=200)

if __name__ == '__main__':
    app.run(debug=True, host='10.60.0.5', port=8080)
