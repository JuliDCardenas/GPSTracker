import os
import time
import logging
import hashlib
import requests
import paho.mqtt.client as mqtt
from datetime import datetime, timezone
from math import radians, sin, cos, sqrt, atan2

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

# ---------- VALIDACIÓN GNSS ----------
MAX_VALID_SPEED_KMH = float(os.getenv("MAX_VALID_SPEED_KMH", "180"))
MIN_VALID_ALTITUDE_M = float(os.getenv("MIN_VALID_ALTITUDE_M", "-9990"))
MIN_DERIVED_DT_SEC = float(os.getenv("MIN_DERIVED_DT_SEC", "2"))
MAX_DERIVED_DT_SEC = float(os.getenv("MAX_DERIVED_DT_SEC", "120"))
MAX_DERIVED_SPEED_KMH = float(os.getenv("MAX_DERIVED_SPEED_KMH", "180"))

QUALITY_ALT_VALID = 1
QUALITY_SPEED_VALID = 2

_last_sent_ts = 0.0
_last_lat = None
_last_lon = None
_last_payload_sig = None
_last_valid_point = None


def empty_to_none(value):
	if value is None:
		return None
	value = str(value).strip()
	return None if value == "" else value


def to_float_or_none(value):
	value = empty_to_none(value)
	if value is None:
		return None
	return float(value)


def parse_ts(ts: str):
	try:
		return datetime.fromisoformat(ts.replace("Z", "+00:00")).astimezone(timezone.utc)
	except Exception:
		return None


def haversine_m(lat1, lon1, lat2, lon2):
	# distancia aproximada en metros
	R = 6371000.0
	p1, p2 = radians(lat1), radians(lat2)
	dlat = radians(lat2 - lat1)
	dlon = radians(lon2 - lon1)
	a = sin(dlat / 2) ** 2 + cos(p1) * cos(p2) * sin(dlon / 2) ** 2
	c = 2 * atan2(sqrt(a), sqrt(1 - a))
	return R * c


def is_valid_speed(speed):
	return speed is not None and 0 <= float(speed) <= MAX_VALID_SPEED_KMH


def is_valid_altitude(alt):
	return alt is not None and float(alt) > MIN_VALID_ALTITUDE_M


def build_quality(speed, alt):
	quality = 0
	if is_valid_altitude(alt):
		quality |= QUALITY_ALT_VALID
	if is_valid_speed(speed):
		quality |= QUALITY_SPEED_VALID
	return quality


def normalize_v1(parts):
	"""v1 histórico: v1,device,fix,lat,lon,speed,alt,vsat,acc,ts"""
	speed = to_float_or_none(parts[5])
	alt = to_float_or_none(parts[6])
	quality = build_quality(speed, alt)

	return {
		"version": "v1",
		"device_id": parts[1],
		"fix": int(parts[2]),
		"lat": float(parts[3]),
		"lon": float(parts[4]),
		"speed": speed if quality & QUALITY_SPEED_VALID else None,
		"raw_speed": speed,
		"alt": alt if quality & QUALITY_ALT_VALID else None,
		"raw_alt": alt,
		"vsat": int(parts[7]),
		"acc": float(parts[8]),
		"quality": quality,
		"ignition": 1,
		"event": "-",
		"ts": parts[9],
		"speed_source": "gps" if quality & QUALITY_SPEED_VALID else "invalid",
	}


def normalize_v2(parts):
	"""v2 actual: v2,device,fix,lat,lon,speed,alt,vsat,acc,quality,ts"""
	quality = int(parts[9])
	speed = to_float_or_none(parts[5])
	alt = to_float_or_none(parts[6])

	return {
		"version": "v2",
		"device_id": parts[1],
		"fix": int(parts[2]),
		"lat": float(parts[3]),
		"lon": float(parts[4]),
		"speed": speed if quality & QUALITY_SPEED_VALID else None,
		"raw_speed": speed,
		"alt": alt if quality & QUALITY_ALT_VALID else None,
		"raw_alt": alt,
		"vsat": int(parts[7]),
		"acc": float(parts[8]),
		"quality": quality,
		"ignition": 1,
		"event": "-",
		"ts": parts[10],
		"speed_source": "gps" if quality & QUALITY_SPEED_VALID else "invalid",
	}


def normalize_v2_legacy_event(parts):
	"""v2 legado del subscriber: v2,device,fix,lat,lon,speed,alt,vsat,acc,ignition,event,ts"""
	speed = to_float_or_none(parts[5])
	alt = to_float_or_none(parts[6])
	quality = build_quality(speed, alt)

	return {
		"version": "v2_legacy_event",
		"device_id": parts[1],
		"fix": int(parts[2]),
		"lat": float(parts[3]),
		"lon": float(parts[4]),
		"speed": speed if quality & QUALITY_SPEED_VALID else None,
		"raw_speed": speed,
		"alt": alt if quality & QUALITY_ALT_VALID else None,
		"raw_alt": alt,
		"vsat": int(parts[7]),
		"acc": float(parts[8]),
		"quality": quality,
		"ignition": int(parts[9]),
		"event": parts[10],
		"ts": parts[11],
		"speed_source": "gps" if quality & QUALITY_SPEED_VALID else "invalid",
	}


def parse_csv(raw: str):
	"""Convierte el CSV del ESP en dict normalizado.

	Compatibilidad:
	- v1 histórico: v1,device,fix,lat,lon,speed,alt,vsat,acc,ts
	- v2 actual:     v2,device,fix,lat,lon,speed,alt,vsat,acc,quality,ts
	- v2 legado con ignition/event: v2,device,fix,lat,lon,speed,alt,vsat,acc,ignition,event,ts
	"""
	parts = raw.split(",")
	if not parts:
		return None

	ver = parts[0].strip()
	try:
		if ver == "v1":
			return normalize_v1(parts)
		if ver == "v2" and len(parts) == 11:
			return normalize_v2(parts)
		if ver == "v2" and len(parts) == 12:
			return normalize_v2_legacy_event(parts)
	except (IndexError, ValueError) as e:
		logging.warning("CSV inválido: %s (%s)", raw[:200], e)
		return None

	logging.warning("Versión/formato CSV desconocido: %s", raw[:120])
	return None


def payload_signature(d: dict) -> str:
	s = f"{d.get('version')}|{d.get('fix')}|{d.get('lat')}|{d.get('lon')}|{d.get('speed')}|{d.get('alt')}|{d.get('quality')}|{d.get('event')}|{d.get('ts')}"
	return hashlib.sha1(s.encode("utf-8")).hexdigest()


def derive_speed_if_needed(d: dict):
	global _last_valid_point

	if d.get("speed") is not None:
		return d

	if _last_valid_point is None:
		return d

	current_ts = parse_ts(d.get("ts", ""))
	previous_ts = parse_ts(_last_valid_point.get("ts", ""))
	if current_ts is None or previous_ts is None:
		logging.info("No se deriva velocidad: timestamp no confiable")
		return d

	dt = (current_ts - previous_ts).total_seconds()
	if dt < MIN_DERIVED_DT_SEC or dt > MAX_DERIVED_DT_SEC:
		logging.info("No se deriva velocidad: dt fuera de rango %.1fs", dt)
		return d

	distance_m = haversine_m(_last_valid_point["lat"], _last_valid_point["lon"], d["lat"], d["lon"])
	derived_speed_kmh = (distance_m / dt) * 3.6
	if derived_speed_kmh < 0 or derived_speed_kmh > MAX_DERIVED_SPEED_KMH:
		logging.info("No se deriva velocidad: %.1f km/h fuera de rango", derived_speed_kmh)
		return d

	d["speed"] = derived_speed_kmh
	d["speed_source"] = "derived"
	logging.info("Velocidad derivada %.1f km/h dist=%.1fm dt=%.1fs", derived_speed_kmh, distance_m, dt)
	return d


def remember_valid_point(d: dict):
	global _last_valid_point
	_last_valid_point = {
		"lat": float(d["lat"]),
		"lon": float(d["lon"]),
		"ts": d.get("ts"),
		"acc": d.get("acc"),
		"vsat": d.get("vsat"),
	}


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

	data = derive_speed_if_needed(data)
	sig = payload_signature(data)
	if is_event:
		# bypass de rate-limit / movimiento mínimo para el aviso de estacionado
		ok, reason = True, f"event:{event} ign={ignition}"
	else:
		ok, reason = should_send(float(lat), float(lon), sig)
	if not ok:
		logging.info("Skip: %s", reason)
		remember_valid_point(data)
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
			remember_valid_point(data)
			logging.info(
				"TRACCAR OK ver=%s lat=%.5f lon=%.5f speed=%s src=%s alt=%s q=%s ign=%s ev=%s",
				data.get("version"), float(lat), float(lon), data.get("speed"),
				data.get("speed_source"), data.get("alt"), data.get("quality"), ignition, event,
			)
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
