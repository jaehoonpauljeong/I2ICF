from django.contrib import admin
from django.urls import path
from intentString.views import InferView, StopView, StatusView, LogView

urlpatterns = [
    path('admin/', admin.site.urls),
    path('api/infer/',  InferView.as_view(),  name='infer'),
    path('api/stop/',   StopView.as_view(),   name='stop'),
    path('api/status/', StatusView.as_view(), name='status'),
    path('api/logs/',   LogView.as_view(),    name='logs'),
]
