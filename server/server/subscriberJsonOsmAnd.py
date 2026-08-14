import os
import time
import logging
import hashlib
import requests
import paho.mqtt.client as mqtt

logging.basicConfig(
	level=os.getenv("LOG_LEVEL", "INFO"),
	format="%(asctime)s %(levelname)s %(message)s",
)

# ---------- MQTT ----------
MQTT_HOST = os.getenv("MQTT_HOST", "mqtt.julidcardenas.site")
MQTT_PORT = int(os.getenv("MQTT_PORT", "1883"))
MQTT_USER = os.getenv("MQTT_USER", "")
MQTT_PASS = os.getenv("MQTT_PASS", "")
MQTT_TOPIC = os.getenv("MQTT_TOPIC", "tracker/Lilygo/telemetria")

# ---------- TRACCAR (OsmAnd) ----------
TRACCAR_HOST = os.getenv("TRACCAR_HOST", "traccar.julidcardenas.site")
TRACCAR_PORT = int(os.getenv("TRACCAR_PORT", "5055"))
TRACCAR_DEVICE_ID = os.getenv("TRACCAR_DEVICE_ID", "Lilygo")

# ---------- CONTROL ----------
# Para no gastar datos ni spamear Traccar
MIN_INTERVAL_SEC = float(os.getenv("MIN_INTERVAL_SEC", "10"))  # no envía más rápido que esto
MIN_MOVE_METERS = float(os.getenv("MIN_MOVE_METERS", "20"))    # si no se movió >X m, no envía
HTTP_TIMEOUT_SEC = float(os.getenv("HTTP_TIMEOUT_SEC", "5"))

_last_sent_ts = 0.0
_last_lat = None
_last_lon = None
_last_payload_sig = None


def haversine_m(lat1, lon1, lat2, lon2):
	# distancia aproximada en metros
	from math import radians, sin, cos, sqrt, atan2
	R = 6371000.0
	p1, p2 = radians(lat1), radians(lat2)
	dlat = radians(lat2 - lat1)
	dlon = radians(lon2 - lon1)
	a = sin(dlat / 2) ** 2 + cos(p1) * cos(p2) * sin(dlon / 2) ** 2
	c = 2 * atan2(sqrt(a), sqrt(1 - a))
	return R * c


def parse_csv(raw: str):
	"""Convierte el CSV del ESP en dict.
	v2: v2,device,fix,lat,lon,speed,alt,vsat,acc,ignition,event,ts
	v1: v1,device,fix,lat,lon,speed,alt,vsat,acc,ts  (compatibilidad)
	"""
	parts = raw.split(",")
	if not parts:
		return None
	ver = parts[0]
	try:
		if ver == "v2":
			return {
				"device_id": parts[1],
				"fix": int(parts[2]),
				"lat": float(parts[3]),
				"lon": float(parts[4]),
				"speed": float(parts[5]),
				"alt": float(parts[6]),
				"vsat": int(parts[7]),
				"acc": float(parts[8]),
				"ignition": int(parts[9]),
				"event": parts[10],
				"ts": parts[11],
			}
		if ver == "v1":
			return {
				"device_id": parts[1],
				"fix": int(parts[2]),
				"lat": float(parts[3]),
				"lon": float(parts[4]),
				"speed": float(parts[5]),
				"alt": float(parts[6]),
				"vsat": int(parts[7]),
				"acc": float(parts[8]),
				"ignition": 1,
				"event": "-",
				"ts": parts[9],
			}
	except (IndexError, ValueError) as e:
		logging.warning("Bad CSV payload: %s (%s)", raw[:200], e)
		return None
	logging.warning("Unknown payload version: %s", raw[:60])
	return None


def payload_signature(d: dict) -> str:
	s = f"{d.get('fix')}|{d.get('lat')}|{d.get('lon')}|{d.get('speed')}|{d.get('alt')}|{d.get('event')}|{d.get('ts')}"
	return hashlib.sha1(s.encode("utf-8")).hexdigest()


def send_osmand(lat, lon, speed=None, alt=None, ignition=None, event=None):
	# MVP: id/lat/lon es suficiente; ignition marca encendido/apagado en Traccar
	params = {
		"id": TRACCAR_DEVICE_ID,
		"lat": f"{lat:.5f}",
		"lon": f"{lon:.5f}",
	}
	if speed is not None:
		params["speed"] = f"{float(speed):.1f}"
	if alt is not None:
		params["altitude"] = f"{float(alt):.0f}"
	if ignition is not None:
		params["ignition"] = "true" if ignition else "false"
	if event and event != "-":
		params["event"] = event

	url = f"http://{TRACCAR_HOST}:{TRACCAR_PORT}/"
	r = requests.get(url, params=params, timeout=HTTP_TIMEOUT_SEC)
	return r.status_code, r.text[:120]


def should_send(lat, lon, sig):
	global _last_sent_ts, _last_lat, _last_lon, _last_payload_sig

	now = time.time()

	# rate limit
	if now - _last_sent_ts < MIN_INTERVAL_SEC:
		return False, "rate_limit"

	# dedupe exacto
	if _last_payload_sig == sig:
		return False, "duplicate"

	# movimiento mínimo
	if _last_lat is not None and _last_lon is not None:
		dist = haversine_m(_last_lat, _last_lon, lat, lon)
		if dist < MIN_MOVE_METERS:
			return False, f"no_move_{dist:.1f}m"

	return True, "ok"


def on_connect(client, userdata, flags, rc):
	if rc == 0:
		logging.info("MQTT connected")
		client.subscribe(MQTT_TOPIC)
		logging.info("Subscribed to %s", MQTT_TOPIC)
	else:
		logging.error("MQTT connect failed rc=%s", rc)


def on_message(client, userdata, msg):
	global _last_sent_ts, _last_lat, _last_lon, _last_payload_sig

	raw = msg.payload.decode("utf-8", errors="replace").strip()
	data = parse_csv(raw)
	if not data:
		return

	lat = data.get("lat")
	lon = data.get("lon")
	if lat is None or lon is None:
		logging.info("Skip: missing lat/lon")
		return

	fix = int(data.get("fix", 0) or 0)
	event = data.get("event", "-")
	ignition = int(data.get("ignition", 1))
	# Un evento de parqueo o ignición apagada NO debe filtrarse por "no se movió"
	is_event = (event and event != "-") or ignition == 0

	if not is_event and fix < 2:
		logging.info("Skip: bad fix=%s", fix)
		return

	sig = payload_signature(data)
	if is_event:
		# bypass de rate-limit / movimiento mínimo para el aviso de estacionado
		ok, reason = True, f"event:{event} ign={ignition}"
	else:
		ok, reason = should_send(float(lat), float(lon), sig)
	if not ok:
		logging.info("Skip: %s", reason)
		return

	try:
		code, preview = send_osmand(
			float(lat), float(lon),
			data.get("speed"), data.get("alt"),
			ignition=ignition, event=event,
		)
		if code == 200:
			_last_sent_ts = time.time()
			_last_lat = float(lat)
			_last_lon = float(lon)
			_last_payload_sig = sig
			logging.info("TRACCAR OK lat=%.5f lon=%.5f ign=%s ev=%s", float(lat), float(lon), ignition, event)
		else:
			logging.warning("TRACCAR HTTP %s body=%s", code, preview)
	except Exception as e:
		logging.warning("TRACCAR send failed: %s", e)


def main():
	client = mqtt.Client()
	if MQTT_USER:
		client.username_pw_set(MQTT_USER, MQTT_PASS)

	client.on_connect = on_connect
	client.on_message = on_message

	logging.info("Connecting MQTT %s:%s topic=%s", MQTT_HOST, MQTT_PORT, MQTT_TOPIC)
	client.connect(MQTT_HOST, MQTT_PORT, keepalive=30)
	client.loop_forever()


if __name__ == "__main__":
	main()
