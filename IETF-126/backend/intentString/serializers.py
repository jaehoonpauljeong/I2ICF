from rest_framework.serializers import ModelSerializer
from .models import IntentLog, ActionLog


class IntentLogSerializer(ModelSerializer):
    class Meta:
        model = IntentLog
        fields = ['id', 'input_text', 'intent_json', 'created_at']


class ActionLogSerializer(ModelSerializer):
    class Meta:
        model = ActionLog
        fields = ['id', 'intent_json', 'steps_json', 'results_json', 'created_at']
