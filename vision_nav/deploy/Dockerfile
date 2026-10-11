FROM python:3.12-slim
WORKDIR /app
COPY server/requirements.txt server/requirements.txt
RUN pip install --no-cache-dir -r server/requirements.txt
COPY server server
COPY web web
WORKDIR /app/server
CMD ["python", "-u", "app.py"]
