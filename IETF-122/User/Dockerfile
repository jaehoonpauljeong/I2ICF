FROM python:3.9

WORKDIR /app

# Python 파일 및 YAML 파일 복사
COPY *.py /app/
COPY intentResult.yaml /app/

# Flask 포함하여 필요한 패키지 설치
RUN pip install requests pyyaml flask

# 실행할 Python 파일 지정
CMD ["python3", "user_flask.py"]
