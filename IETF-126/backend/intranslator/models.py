from django.db import models

class Intent(models.Model):
    user_label = models.CharField(max_length=255)
    expectation_id = models.CharField(max_length=255)
    expectation_verb = models.CharField(max_length=50)
    object_type = models.CharField(max_length=50)
    context_attributes = models.JSONField()
    target_metrics = models.JSONField()
    priority = models.CharField(max_length=50)
    location = models.CharField(max_length=50, null=True, blank=True)  # 수정됨
    observation_period = models.CharField(max_length=50)
    report_reference = models.CharField(max_length=255)

    def __str__(self):
        return self.user_label
