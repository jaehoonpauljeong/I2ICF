FROM python:3.9

WORKDIR /app

# Python 파일 복사
COPY *.py /app/

# Flask 및 필요한 패키지 설치
RUN pip install flask requests pytz

# 실행할 Python 파일 지정
CMD ["python3", "sdv_flask.py"]
