# @csrf_exempt
# def create_intent(intent_id, user_label, expectation_id, expectation_verb, object_type, context_attributes, target_metrics, priority, observation_period, report_reference):
#     intent = {
#         'Intent': {
#             'Id': intent_id,
#             'userLabel': user_label,
#             'intentExpectation': [
#                 {
#                     'expectationId': expectation_id,
#                     'expectationVerb': expectation_verb,
#                     'expectationObjects': [
#                         {
#                             'objectType': object_type,
#                             'objectContexts': context_attributes
#                         }
#                     ],
#                     'expectationTargets': target_metrics
#                 }
#             ],
#             'intentPriority': priority,
#             'observationPeriod': observation_period,
#             'intentReportReference': report_reference
#         }
#     }
#     return intent


from django.http import HttpResponse
from django.views.decorators.csrf import csrf_exempt
from django.http import JsonResponse
from django.shortcuts import render
import requests
from .models import Intent
import yaml
import json
import os
def home(request):
    return HttpResponse("<h1>Welcome to the Intent Translator API</h1><p>Use /api/upload/ to upload your intent data.</p>")
@csrf_exempt
# def upload_file(request):
#     if request.method == 'PUT':
#         try:
#             # Parse the incoming JSON data
#             data = json.loads(request.body)

#             # Convert the JSON data to YAML
#             yaml_data = yaml.dump(data, default_flow_style=False)
#             print(yaml_data)
#             # Send YAML data to external virtual server
#             external_server_url = 'http://192.0.2.1:5000/upload'  # Replace with actual external server IP address
#             response = requests.post(external_server_url, data=yaml_data, headers={'Content-Type': 'application/x-yaml'})

#             # Check response from external server
#             if response.status_code == 200:
#                 return HttpResponse(f"<pre>YAML data successfully sent to external server. Response: {response.text}</pre>", content_type="text/html")
#             else:
#                 return HttpResponse(f"<pre>Failed to send YAML data to external server. Response Code: {response.status_code} Response: {response.text}</pre>", content_type="text/html")
#         except Exception as e:
#             return HttpResponse(f"<pre>Failed to process file content: {str(e)}</pre>", content_type="text/html")
#     else:
#         return HttpResponse("<pre>Invalid request method. Please use POST.</pre>", content_type="text/html")


def upload_file(request):
    if request.method == 'POST':
        try:
            # Log the incoming request body for debugging
            print(f"Received request body: {request.body}")

            # Parse the incoming JSON data
            data = json.loads(request.body)

            # Save data to the database
            intent = Intent.objects.create(
                user_label=data['user_label'],
                expectation_id=data['expectation_id'],
                expectation_verb=data['expectation_verb'],
                object_type=data['object_type'],
                context_attributes=data['context_attributes'],
                target_metrics=data['target_metrics'],
                priority=data['priority'],
                location=data['location'],
                observation_period=data['observation_period'],
                report_reference=data['report_reference']
            )

            # Convert the JSON data to YAML
            yaml_data = yaml.dump(data, default_flow_style=False)
            print(f"YAML data: {yaml_data}")
            # Write YAML data to a file
            with open('intent.yaml', 'w') as yaml_file:
                yaml_file.write(yaml_data)

            # Deliver the YAML file to the virtual server
            with open('intent.yaml', 'rb') as f:
                response = requests.post('VIRTUALSERVER_IPADDRESS/UPLOAD', files={'file': f})
                if response.status_code == 200:
                    return JsonResponse({'yaml_data': yaml_data, 'message': 'YAML file delivered to virtual server successfully.'})
                else:
                    return JsonResponse({'yaml_data': yaml_data, 'error': 'Failed to deliver YAML file to virtual server.'}, status=500)

        except Exception as e:
            print(f"Error: {e}")
            return JsonResponse({'error': f'Failed to process file content: {str(e)}'}, status=400)
    else:
        print(f"Invalid request method: {request.method}")
        return JsonResponse({'error': 'Invalid request method.'}, status=405)

@csrf_exempt
def get_intents(request):
    if request.method == 'GET':
        intents = Intent.objects.all().values()
        return JsonResponse(list(intents), safe=False)
    else:
        return JsonResponse({'error': 'Invalid request method.'}, status=405)