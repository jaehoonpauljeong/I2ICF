import requests  
import time
import json
import os

# Kubernetes 내에서 controller 서비스의 이름을 사용하여 접근
CloudServer_IP = "10.152.183.236"  # 변경된 부분
url = f"http://{CloudServer_IP}:5000/send_message"

# intentResult.yaml 파일 경로 (컨테이너 내부 경로)
INTENT_FILE_PATH = "/app/intentResult.yaml"  # 변경된 부분

# Define a function to generate and send XML messages.
def generate_and_send_message():
    while True:
        # Open and read the intentResult.yaml file
        try:
            with open(INTENT_FILE_PATH, 'r', encoding='utf-8') as f:
                intent_str = f.read()
        except FileNotFoundError:
            print(f"File not found: {INTENT_FILE_PATH}")
            time.sleep(1)
            continue

        # Print the intent data to be sent
        print("Generated Intent Data:")
        print(intent_str)
        json_data = json.loads(intent_str)

        # Send the data to the controller service using a POST request
        try:
            response = requests.post(url, json=json_data)
            print(f"Status Code: {response.status_code}")
            print("Response:")
            print(response.text)
        except requests.exceptions.RequestException as e:
            print(f"Request failed: {e}")

        # Wait for 1 second before sending the next message
        time.sleep(1)

# Call the function to execute.
generate_and_send_message()
