import hashlib
import json
import logging
import os
import re
import time

import paho.mqtt.client as mqtt
import psycopg

logging.basicConfig(
    level=os.getenv("LOG_LEVEL", "INFO"),
    format="%(asctime)s %(levelname)s %(message)s",
)

MQTT_HOST = os.getenv("MQTT_HOST", "mosquitto")
MQTT_PORT = int(os.getenv("MQTT_PORT", "1883"))
MQTT_USER = os.getenv("MQTT_USER", "")
MQTT_PASS = os.getenv("MQTT_PASS", "")
MQTT_TOPICS = tuple(
    topic.strip()
    for topic in os.getenv(
        "MQTT_TOPICS",
        "tracker/Lilygo/sys/#,tracker/Lilygo/event/#,tracker/Lilygo/ack/#",
    ).split(",")
    if topic.strip()
)
DEVICE_ID = os.getenv("DEVICE_ID", "Lilygo")

PGHOST = os.getenv("PGHOST", "mqtt-postgres")
PGPORT = int(os.getenv("PGPORT", "5432"))
PGDATABASE = os.getenv("PGDATABASE", "")
PGUSER = os.getenv("PGUSER", "")
PGPASSWORD = os.getenv("PGPASSWORD", "")
PGSCHEMA = os.getenv("PGSCHEMA", "tracker_diag")
MAX_PAYLOAD_CHARS = int(os.getenv("MAX_PAYLOAD_CHARS", "4096"))

LOCATION_KEYS = {"lat", "lon", "latitude", "longitude"}
conn = None


def validate_schema_name(name):
    if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", name):
        raise ValueError("PGSCHEMA contains unsupported characters")
    return name


def sanitize_value(value):
    if isinstance(value, dict):
        return {
            key: sanitize_value(item)
            for key, item in value.items()
            if key.lower() not in LOCATION_KEYS
        }
    if isinstance(value, list):
        return [sanitize_value(item) for item in value]
    return value


def sanitize_payload(raw):
    try:
        parsed = json.loads(raw)
    except json.JSONDecodeError:
        return raw[:MAX_PAYLOAD_CHARS]
    sanitized = sanitize_value(parsed)
    return json.dumps(
        sanitized,
        ensure_ascii=False,
        separators=(",", ":"),
    )[:MAX_PAYLOAD_CHARS]


def message_type(topic):
    prefix = f"tracker/{DEVICE_ID}/"
    return topic[len(prefix):] if topic.startswith(prefix) else topic


def pg_connect():
    connection = psycopg.connect(
        host=PGHOST,
        port=PGPORT,
        dbname=PGDATABASE,
        user=PGUSER,
        password=PGPASSWORD,
    )
    connection.autocommit = True
    return connection


def connect_with_retry(max_attempts=30):
    global conn
    for attempt in range(1, max_attempts + 1):
        try:
            conn = pg_connect()
            logging.info("PostgreSQL connected")
            return
        except Exception as exc:
            if attempt == max_attempts:
                raise
            logging.warning(
                "PostgreSQL unavailable attempt=%s/%s error=%s",
                attempt,
                max_attempts,
                exc,
            )
            time.sleep(min(attempt, 10))


def init_schema():
    schema = validate_schema_name(PGSCHEMA)
    with conn.cursor() as cur:
        cur.execute(f"CREATE SCHEMA IF NOT EXISTS {schema}")
        cur.execute(
            f"""
            CREATE TABLE IF NOT EXISTS {schema}.messages (
                id BIGINT GENERATED ALWAYS AS IDENTITY PRIMARY KEY,
                received_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
                device_id TEXT NOT NULL,
                topic TEXT NOT NULL,
                message_type TEXT NOT NULL,
                payload TEXT NOT NULL,
                payload_sha256 TEXT NOT NULL,
                payload_bytes INTEGER NOT NULL,
                qos SMALLINT NOT NULL,
                retained BOOLEAN NOT NULL
            )
            """
        )
        cur.execute(
            f"CREATE INDEX IF NOT EXISTS messages_device_time_idx "
            f"ON {schema}.messages (device_id, received_at DESC)"
        )
        cur.execute(
            f"CREATE INDEX IF NOT EXISTS messages_topic_time_idx "
            f"ON {schema}.messages (topic, received_at DESC)"
        )
        cur.execute(
            f"""
            CREATE TABLE IF NOT EXISTS {schema}.states (
                device_id TEXT NOT NULL,
                topic TEXT NOT NULL,
                last_seen_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
                payload TEXT NOT NULL,
                payload_sha256 TEXT NOT NULL,
                payload_bytes INTEGER NOT NULL,
                PRIMARY KEY (device_id, topic)
            )
            """
        )
    logging.info("Schema %s ready", schema)


def persist_message(msg, payload):
    global conn
    schema = validate_schema_name(PGSCHEMA)
    digest = hashlib.sha256(payload.encode("utf-8")).hexdigest()
    values = (
        DEVICE_ID,
        msg.topic,
        message_type(msg.topic),
        payload,
        digest,
        len(msg.payload),
        msg.qos,
        bool(msg.retain),
    )

    for attempt in range(2):
        try:
            with conn.cursor() as cur:
                cur.execute(
                    f"""
                    INSERT INTO {schema}.messages
                        (device_id, topic, message_type, payload,
                         payload_sha256, payload_bytes, qos, retained)
                    VALUES (%s, %s, %s, %s, %s, %s, %s, %s)
                    """,
                    values,
                )
                cur.execute(
                    f"""
                    INSERT INTO {schema}.states
                        (device_id, topic, last_seen_at, payload,
                         payload_sha256, payload_bytes)
                    VALUES (%s, %s, NOW(), %s, %s, %s)
                    ON CONFLICT (device_id, topic) DO UPDATE SET
                        last_seen_at = EXCLUDED.last_seen_at,
                        payload = EXCLUDED.payload,
                        payload_sha256 = EXCLUDED.payload_sha256,
                        payload_bytes = EXCLUDED.payload_bytes
                    """,
                    (DEVICE_ID, msg.topic, payload, digest, len(msg.payload)),
                )
            return
        except Exception as exc:
            logging.warning(
                "PostgreSQL write failed attempt=%s error=%s",
                attempt + 1,
                exc,
            )
            if attempt == 0:
                try:
                    conn.close()
                except Exception:
                    pass
                connect_with_retry(max_attempts=5)
            else:
                raise


def on_connect(client, userdata, flags, rc):
    if rc != 0:
        logging.error("MQTT connect failed rc=%s", rc)
        return
    logging.info("MQTT connected")
    for topic in MQTT_TOPICS:
        client.subscribe(topic, qos=0)
        logging.info("Subscribed %s", topic)


def on_disconnect(client, userdata, rc):
    if rc:
        logging.warning("MQTT disconnected unexpectedly rc=%s", rc)


def on_message(client, userdata, msg):
    raw = msg.payload.decode("utf-8", errors="replace").strip()
    payload = sanitize_payload(raw)
    try:
        persist_message(msg, payload)
        logging.info(
            "Stored topic=%s bytes=%s retained=%s",
            msg.topic,
            len(msg.payload),
            bool(msg.retain),
        )
    except Exception as exc:
        logging.error("Message lost topic=%s error=%s", msg.topic, exc)


def main():
    validate_schema_name(PGSCHEMA)
    connect_with_retry()
    init_schema()

    client = mqtt.Client(client_id="tracker-diagnostics-v1")
    if MQTT_USER:
        client.username_pw_set(MQTT_USER, MQTT_PASS)
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_message = on_message

    logging.info("Connecting MQTT %s:%s", MQTT_HOST, MQTT_PORT)
    client.connect(MQTT_HOST, MQTT_PORT, keepalive=60)
    client.loop_forever()


if __name__ == "__main__":
    main()
