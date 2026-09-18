FROM python:3.13-slim
WORKDIR /app
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt
COPY subscriberJsonOsmAnd.py event_store.py ./
RUN mkdir -p /data
ENV PYTHONUNBUFFERED=1
ENV EVENT_DB_PATH=/data/event_state.sqlite3
CMD ["python", "subscriberJsonOsmAnd.py"]
