import json
import logging
import os

import paho.mqtt.client as mqtt

import subscriberJsonOsmAnd as legacy
from event_store import EventStore

MQTT_EVENT_TOPIC = os.getenv("MQTT_EVENT_TOPIC", "tracker/Lilygo/event/ignition/v3")
MQTT_EVENT_ACK_TOPIC = os.getenv("MQTT_EVENT_ACK_TOPIC", "tracker/Lilygo/ack/ignition/v3")
EVENT_DB_PATH = os.getenv("EVENT_DB_PATH", "/data/event_state.sqlite3")

store = None
_original_remember_valid_point = legacy.remember_valid_point


def remember_valid_point_durable(data):
    _original_remember_valid_point(data)
    if store is not None:
        store.save_state("last_valid_point", legacy._last_valid_point)
        process_pending_events()


legacy.remember_valid_point = remember_valid_point_durable


def event_position(payload):
    position = payload.get("position")
    if isinstance(position, dict) and position.get("lat") is not None and position.get("lon") is not None:
        return position, payload.get("position_source", "cache")
    if legacy._last_valid_point is not None:
        return legacy._last_valid_point, "server_cache"
    return None, "none"


def process_pending_events():
    if store is None:
        return
    for event_id, payload in store.pending_events():
        position, delivery_source = event_position(payload)
        if position is None:
            logging.warning("EVENT pending id=%s type=%s: no hay posición para Traccar", event_id, payload.get("type"))
            continue
        try:
            code, preview = legacy.send_osmand(
                float(position["lat"]), float(position["lon"]),
                position.get("speed"), position.get("alt"),
                ignition=int(payload["ignition"]), event=payload["type"],
            )
            if code == 200:
                store.mark_delivered(event_id)
                logging.info(
                    "TRACCAR EVENT OK id=%s type=%s ign=%s event_pos=%s delivery_pos=%s fix_age=%s",
                    event_id, payload.get("type"), payload.get("ignition"),
                    payload.get("position_source"), delivery_source, payload.get("fix_age_s"),
                )
            else:
                logging.warning("TRACCAR EVENT HTTP %s id=%s body=%s", code, event_id, preview)
                break
        except Exception as exc:
            logging.warning("TRACCAR EVENT failed id=%s: %s", event_id, exc)
            break


def handle_ignition_event(client, raw):
    try:
        payload = json.loads(raw)
        event_id = str(payload["event_id"])
        event_type = payload["type"]
        ignition = int(payload["ignition"])
        if event_type not in ("engine_on", "engine_off") or ignition not in (0, 1):
            raise ValueError("type/ignition inválidos")
    except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
        logging.warning("EVENT inválido: %s (%s)", raw[:240], exc)
        return

    inserted, status = store.accept_event(payload)
    logging.info("EVENT %s id=%s type=%s status=%s", "nuevo" if inserted else "duplicado", event_id, event_type, status)
    info = client.publish(MQTT_EVENT_ACK_TOPIC, event_id, qos=1, retain=False)
    if info.rc != mqtt.MQTT_ERR_SUCCESS:
        logging.warning("EVENT ACK publish failed id=%s rc=%s", event_id, info.rc)
    process_pending_events()


def on_connect(client, userdata, flags, rc):
    if rc != 0:
        logging.error("MQTT connect failed rc=%s", rc)
        return
    logging.info("MQTT connected")
    client.subscribe(legacy.MQTT_TOPIC)
    client.subscribe(MQTT_EVENT_TOPIC, qos=1)
    if legacy.LWT_TRACCAR_MODE != "off":
        client.subscribe(legacy.MQTT_LWT_TOPIC)
    logging.info("Subscribed telemetry=%s events=%s", legacy.MQTT_TOPIC, MQTT_EVENT_TOPIC)


def on_message(client, userdata, msg):
    raw = msg.payload.decode("utf-8", errors="replace").strip()
    if msg.topic == legacy.MQTT_LWT_TOPIC:
        legacy.handle_lwt(raw)
    elif msg.topic == MQTT_EVENT_TOPIC:
        handle_ignition_event(client, raw)
    else:
        legacy.handle_telemetry(raw)


def main():
    global store
    os.makedirs(os.path.dirname(EVENT_DB_PATH) or ".", exist_ok=True)
    store = EventStore(EVENT_DB_PATH)
    legacy._last_valid_point = store.load_state("last_valid_point")
    process_pending_events()

    client = mqtt.Client()
    if legacy.MQTT_USER:
        client.username_pw_set(legacy.MQTT_USER, legacy.MQTT_PASS)
    client.on_connect = on_connect
    client.on_message = on_message
    logging.info("Connecting MQTT %s:%s telemetry=%s events=%s", legacy.MQTT_HOST, legacy.MQTT_PORT, legacy.MQTT_TOPIC, MQTT_EVENT_TOPIC)
    client.connect(legacy.MQTT_HOST, legacy.MQTT_PORT, keepalive=30)
    client.loop_forever()


if __name__ == "__main__":
    main()
