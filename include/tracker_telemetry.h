#pragma once
// tracker_telemetry.h — lectura GNSS y entrega transaccional de eventos.
// Se incluye desde main.cpp después de serviceMQTT().

static RTC_DATA_ATTR uint32_t rtcFixAgeS = 0;
static uint32_t fixAgeBaseMs = 0;
static inline uint32_t fixAgeS() { return rtcFixAgeS + ((millis() - fixAgeBaseMs) / 1000); }

#define FIX_MAX_AGE_S 300UL
#define EVENT_RETRY_MS 5000UL
#define EVENT_RETRY_OFF_NO_POINT_MS 30000UL

static bool readGpsPoint(GpsPoint &out) {
  float lat = 0, lon = 0, speed = 0, alt = 0, acc = 0;
  int vsat = 0, usat = 0;
  int year = 0, month = 0, day = 0, hour = 0, min = 0, sec = 0;
  uint8_t fix = 0;

  bool ok = modem.getGPS(&fix, &lat, &lon, &speed, &alt, &vsat, &usat, &acc,
                         &year, &month, &day, &hour, &min, &sec);
  if (!ok) { SerialMon.println("[GPS] read FAIL"); return false; }
  if (!isGpsPositionValid(fix, lat, lon, vsat, acc)) {
    SerialMon.printf("[GPS] posición inválida fix=%u lat=%.6f lon=%.6f speed=%.2f alt=%.1f sats=%d hdop=%.2f\n",
                     fix, lat, lon, speed, alt, vsat, acc);
    return false;
  }
  if (!isGpsTimeValid(year, month, day, hour, min, sec)) {
    SerialMon.printf("[GPS] trama corrupta, fecha %04d-%02d-%02dT%02d:%02d:%02d -> descartada\n",
                     year, month, day, hour, min, sec);
    return false;
  }

  char tsNow[24];
  snprintf(tsNow, sizeof(tsNow), "%04d-%02d-%02dT%02d:%02d:%02dZ",
           year, month, day, hour, min, sec);
  if (gnssFrameIsStale(tsNow, lastValidPoint.valid, lastValidPoint.ts)) {
    if (mqtt.connected()) {
      char g[64];
      snprintf(g, sizeof(g), "stale=%u,last=%s", (unsigned)rtcGnssStale, tsNow);
      mqtt.publish(TOPIC_GNSS, g, true);
    }
    return false;
  }

  gnssNoteFirstFix(vsat, acc);
  gnssLogSpeedUnits(speed);
  uint8_t quality = buildGpsQuality(speed, alt);

  out.valid = true; out.fix = fix; out.lat = lat; out.lon = lon;
  out.speed = speed; out.alt = alt; out.acc = acc; out.vsat = vsat;
  out.speedValid = (quality & GPS_QUALITY_SPEED_VALID) != 0;
  out.altValid = (quality & GPS_QUALITY_ALT_VALID) != 0;
  snprintf(out.ts, sizeof(out.ts), "%s", tsNow);
  return true;
}

static void rememberPoint(const GpsPoint &p) {
  rtcFixAgeS = 0;
  fixAgeBaseMs = millis();
  lastValidPoint = p;
  if (p.speedValid && p.speed > MOVING_SPEED_KMH) lastMovementMs = millis();
}

static bool publishPointWithIgnition(const GpsPoint &p, const char *event, uint8_t ignition) {
  if (!mqtt.connected()) { SerialMon.println("[PUB] omitido: MQTT desconectado"); return false; }
  if (!p.valid) { SerialMon.println("[PUB] omitido: aún no hay posición válida en caché"); return false; }

  char speedField[16] = "", altField[16] = "";
  if (p.speedValid) snprintf(speedField, sizeof(speedField), "%.2f", p.speed);
  if (p.altValid) snprintf(altField, sizeof(altField), "%.1f", p.alt);

  char payload[256];
  snprintf(payload, sizeof(payload),
           "v2,%s,%u,%.6f,%.6f,%s,%s,%d,%.2f,%u,%s,%s",
           DEVICE_ID, p.fix, p.lat, p.lon, speedField, altField, p.vsat, p.acc,
           ignition, event, p.ts);
  bool ok = mqtt.publish(TOPIC_TELEMETRY, payload);
  SerialMon.printf("[PUB] %s topic=%s payload=%s\n", ok ? "OK" : "FAIL", TOPIC_TELEMETRY, payload);
  return ok;
}

static bool publishPoint(const GpsPoint &p, const char *event) {
  return publishPointWithIgnition(p, event, ignitionField());
}

static void serviceEvents() {
  if (pendingEvent == EV_NONE || !mqtt.connected()) return;

  static uint32_t lastEventAttemptMs = 0;
  uint32_t now = millis();
  uint32_t retryMs = (pendingEvent == EV_ENGINE_OFF && !lastValidPoint.valid)
                         ? EVENT_RETRY_OFF_NO_POINT_MS
                         : EVENT_RETRY_MS;
  if (lastEventAttemptMs != 0 && (now - lastEventAttemptMs) < retryMs) return;
  lastEventAttemptMs = now;

  const bool isOn = (pendingEvent == EV_ENGINE_ON);
  const char *eventName = isOn ? EVENT_ENGINE_ON : EVENT_ENGINE_OFF;
  const uint8_t eventIgnition = isOn ? 1 : 0;
  if (isOn) lastMovementMs = now;

  GpsPoint fresh = {};
  if (readGpsPoint(fresh)) rememberPoint(fresh);

  if (!lastValidPoint.valid) {
    SerialMon.printf("[PUB] evento %s pendiente: sin posición válida; MQTT permanece conectado\n",
                     eventName);
    return;
  }

  if (isOn && fixAgeS() > FIX_MAX_AGE_S) {
    SerialMon.printf("[PUB] engine_on diferido: caché=%lus, esperando fix fresco\n",
                     (unsigned long)fixAgeS());
    return;
  }

  bool stateDelivered = mqtt.publish(TOPIC_IGNITION, isOn ? "on" : "off", true);
  bool pointDelivered = publishPointWithIgnition(lastValidPoint, eventName, eventIgnition);
  if (stateDelivered && pointDelivered) {
    pendingEvent = EV_NONE;
    lastPublishMs = millis();
    SerialMon.printf("[PUB] evento %s confirmado y limpiado\n", eventName);
  } else {
    SerialMon.printf("[PUB] evento %s pendiente: state=%d point=%d; sin cerrar socket aquí\n",
                     eventName, (int)stateDelivered, (int)pointDelivered);
  }
}

static void serviceTelemetry() {
  static uint32_t lastTelemetryAttemptMs = 0;
  uint32_t now = millis();
  if ((now - lastTelemetryAttemptMs) < currentPeriodMs()) return;
  lastTelemetryAttemptMs = now;

  GpsPoint fresh = {};
  bool haveFresh = readGpsPoint(fresh);
  if (haveFresh) rememberPoint(fresh);

  bool sent = false;
  if (haveFresh) sent = publishPoint(lastValidPoint, EVENT_NONE);
  else if (isIgnitionOff()) sent = publishPoint(lastValidPoint, EVENT_NONE);
  if (sent) lastPublishMs = now;
}

static void serviceBattery() {
  uint32_t period = isIgnitionOff() ? (PARKED_KEEPALIVE_MIN * 60000UL) : BATTERY_PERIOD_MS;
  uint32_t now = millis();
  if (batteryEverPublished && (now - lastBatteryMs) < period) return;
  if (!mqtt.connected()) return;

  float volts = readBatteryVolts();
  char buf[16];
  snprintf(buf, sizeof(buf), "%.2f", volts);
  if (mqtt.publish(TOPIC_BATTERY, buf, true)) {
    lastBatteryMs = now;
    batteryEverPublished = true;
  }
  SerialMon.printf("[BAT] %s V\n", buf);
}
