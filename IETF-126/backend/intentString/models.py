from django.db import models


class IntentLog(models.Model):
    input_text = models.TextField()
    intent_json = models.TextField()
    created_at = models.DateTimeField(auto_now_add=True)

    class Meta:
        ordering = ['-created_at']

    def __str__(self):
        return f"IntentLog({self.id}): {self.input_text[:40]}"


class ActionLog(models.Model):
    intent_json = models.TextField()
    steps_json = models.TextField()
    results_json = models.TextField()
    created_at = models.DateTimeField(auto_now_add=True)

    class Meta:
        ordering = ['-created_at']

    def __str__(self):
        return f"ActionLog({self.id}) @ {self.created_at}"
