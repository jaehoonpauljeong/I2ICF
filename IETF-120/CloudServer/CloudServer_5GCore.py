from flask import Flask, request, Response
import xml.etree.ElementTree as ET
import xml.dom.minidom
import requests
import threading

app = Flask(__name__)

vehicle_database = {
    "vehicle_1": {"ip_address": "10.60.0.1", "port": 8080},
    "vehicle_2": {"ip_address": "10.60.0.2", "port": 8081}
}

cloud_server_info = {"ip_address": "203.0.113.1", "port": 9090}

def create_policy_xml(src_ip, dst_ip, action):
    policy = ET.Element('policy')
    ET.SubElement(policy, 'src').text = src_ip
    ET.SubElement(policy, 'dst').text = dst_ip
    ET.SubElement(policy, 'action').text = action
    return ET.tostring(policy, encoding='unicode')

def pretty_print_xml(xml_str):
    xml_dom = xml.dom.minidom.parseString(xml_str)
    return xml_dom.toprettyxml()

def send_message_to_vehicle(vehicle_id, message):
    vehicle_info = vehicle_database.get(vehicle_id)
    if vehicle_info:
        url = f"http://{vehicle_info['ip_address']}:{vehicle_info['port']}/receive_message"
        try:
            response = requests.post(url, data=message, headers={'Content-Type': 'application/xml'})
            print(f"Sent message to {vehicle_id} at {url}: {response.status_code}")
            print(pretty_print_xml(message))
        except requests.exceptions.RequestException as e:
            print(f"Failed to send message to {vehicle_id} at {url}: {e}")

@app.route('/send_message', methods=['POST'])
def send_message():
    file = request.files['file']
    xml_data = file.read().decode('utf-8')

    print("Received XML:")
    print(xml_data)
    print()

    # 메시지 생성
    message1 = create_policy_xml("10.60.0.1", cloud_server_info["ip_address"], "remote-speed")
    message2 = create_policy_xml("10.60.0.2", cloud_server_info["ip_address"], "remote-speed")

    # 메시지 전송 스레드 생성
    threads = []
    threads.append(threading.Thread(target=send_message_to_vehicle, args=("vehicle_1", message1)))
    threads.append(threading.Thread(target=send_message_to_vehicle, args=("vehicle_2", message2)))

    # 스레드 시작
    for thread in threads:
        thread.start()

    # 모든 스레드가 종료될 때까지 대기
    for thread in threads:
        thread.join()

    return Response("Messages processed", mimetype='text/plain'), 200

if __name__ == '__main__':
    app.run(debug=True, host='10.60.0.4', port=5000)
