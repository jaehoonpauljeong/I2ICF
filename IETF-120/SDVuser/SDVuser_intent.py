import requests
import datetime
import pytz
from xml.etree.ElementTree import Element, SubElement, tostring
from xml.dom import minidom

url = "http://10.60.0.4:5000/send_message"

def generate_and_send_message():
    # XML 데이터 생성
    root = Element('policy')
    srcs = SubElement(root, 'srcs')
    src = SubElement(srcs, 'src')
    src.text = "vehicle_1"
    src = SubElement(srcs, 'src')
    src.text = "vehicle_2"
    dst = SubElement(root, 'dst')
    dst.text = "cloud-server"
    action = SubElement(root, 'action')
    action.text = "remote-speed"  # 여기를 수정했습니다. dst.text -> action.text

    # XML을 문자열로 변환
    xml_str = minidom.parseString(tostring(root)).toprettyxml(indent="  ")

    print("Generated XML:")
    print(xml_str)

    # XML 데이터를 파일처럼 전송하기 위해 스트림으로 변환합니다.
    files = {'file': ('message.xml', xml_str, 'application/xml')}

    try:
        response = requests.post(url, files=files, timeout=1)  # 타임아웃 설정
        print(f"Status Code: {response.status_code}")
        print("\nResponse:")
        print(response.text)  # XML 응답을 그대로 출력
    except requests.exceptions.RequestException as e:
        print(f"Request failed: {e}")

    print(files)

# 실행
generate_and_send_message()
