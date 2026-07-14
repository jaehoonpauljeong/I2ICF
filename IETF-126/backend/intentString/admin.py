from django.contrib import admin
from .models import IntentLog, ActionLog

admin.site.register(IntentLog)
admin.site.register(ActionLog)
