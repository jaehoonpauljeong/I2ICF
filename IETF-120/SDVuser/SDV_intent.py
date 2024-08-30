import requests  
import time
from xml.etree.ElementTree import Element, SubElement, tostring
from xml.dom import minidom

# Define the Cloud Server IP and construct the URL for sending messages.
CloudServer_IP = "10.60.0.10"
url = f"http://{CloudServer_IP}:5000/send_message"

# Define a function to generate and send XML messages.
def generate_and_send_message():
    while True:
        # Create the root element of the XML document.
        root = Element('policy')
        # Create a nested element to hold sources.
        srcs = SubElement(root, 'srcs')
        # Add individual source elements and set their text.
        src = SubElement(srcs, 'src')
        src.text = "vehicle_1"
        src = SubElement(srcs, 'src')
        src.text = "vehicle_2"
        # Create a destination element and set its text.
        dst = SubElement(root, 'dst')
        dst.text = "cloud-server"
        # Create an action element and set its text.
        action = SubElement(root, 'action')
        action.text = "remote-speed"

        # Convert the XML tree to a string and pretty print it.
        xml_str = minidom.parseString(tostring(root)).toprettyxml(indent="  ")

        # Print the generated XML string.
        print("Generated XML:")
        print(xml_str)

        # Convert the XML data to a file-like stream for posting.
        files = {'file': ('message.xml', xml_str, 'application/xml')}

        # Try to post the XML data to the server using a POST request.
        try:
            response = requests.post(url, files=files)  # Send the file with the POST r>
            print(f"Status Code: {response.status_code}")  # Print the status code of t>
            print("\nResponse:")
            print(response.text)  # Print the server's response.
        except requests.exceptions.RequestException as e:
            print(f"Request failed: {e}")  # Handle and print any errors during the req>

        # Wait for 1 second before sending the next message.
        time.sleep(1)

# Call the function to execute.
generate_and_send_message()

