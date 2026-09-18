#pragma once
// GNSS, telemetría y eventos de ignición v3 idempotentes.

#include <esp_system.h>
#include <string.h>

static RTC_DATA_ATTR uint32_t rtcFixAgeS = 0;
static uint32_t fixAgeBaseMs = 0;
static inline uint32_t fixAgeS() { return rtcFixAgeS + ((millis() - fixAgeBaseMs) / 1000); }

#define FIX_MAX_AGE_S 300UL
#define EVENT_FRESH_POSITION_MAX_AGE_S 10UL
#define EVENT_RETRY_MS 5000UL
#define EVENT_QUEUE_CAPACITY 8
#define EVENT_MQTT_BUFFER_SIZE 768

static const char TOPIC_IGNITION_STATE_V3[] = "tracker/Lilygo/state/ignition";
static const char TOPIC_IGNITION_EVENT_V3[] = "tracker/Lilygo/event/ignition/v3";
static const char TOPIC_IGNITION_ACK_V3[] = "tracker/Lilygo/ack/ignition/v3";

enum EventPositionSource : uint8_t {
  EVENT_POSITION_NONE = 0,
  EVENT_POSITION_FRESH = 1,
  EVENT_POSITION_CACHE = 2,
};

struct PendingEventRecord {
  bool used;
  bool acknowledged;
  PendingEvent type;
  uint8_t ignition;
  uint32_t sequence;
  uint32_t bootNonce;
  uint32_t occurredMs;
  uint32_t fixAgeAtEventS;
  EventPositionSource positionSource;
  GpsPoint position;
  uint16_t attempts;
  char eventId[48];
};

static RTC_DATA_ATTR PendingEventRecord eventQueue[EVENT_QUEUE_CAPACITY] = {};
static RTC_DATA_ATTR uint8_t eventQueueHead = 0;
static RTC_DATA_ATTR uint8_t eventQueueCount = 0;
static RTC_DATA_ATTR uint32_t eventBootNonce = 0;
static RTC_DATA_ATTR uint32_t eventSequence = 0;
static RTC_DATA_ATTR PendingEvent lastCapturedPendingEvent = EV_NONE;
static uint32_t lastEventAttemptMs = 0;
static bool eventMqttWasConnected = false;

static inline uint8_t eventQueueIndex(uint8_t offset) {
  return (uint8_t)((eventQueueHead + offset) % EVENT_QUEUE_CAPACITY);
}
static inline bool eventStoreEmpty() { return eventQueueCount == 0; }
static inline uint8_t eventStoreCount() { return eventQueueCount; }
static const char *eventTypeName(PendingEvent type) {
  return type == EV_ENGINE_ON ? EVENT_ENGINE_ON : EVENT_ENGINE_OFF;
}
static const char *eventPositionSourceName(EventPositionSource source) {
  if (source == EVENT_POSITION_FRESH) return "fresh";
  if (source == EVENT_POSITION_CACHE) return "cache";
  return "none";
}

static bool enqueueIgnitionEvent(PendingEvent type) {
  if (type == EV_NONE) return false;
  if (eventQueueCount >= EVENT_QUEUE_CAPACITY) {
    SerialMon.printf("[EVENT] cola llena (%u); %s sigue pendiente\n",
                     (unsigned)EVENT_QUEUE_CAPACITY, eventTypeName(type));
    publishStatus("event_queue_full");
    return false;
  }
  if (eventBootNonce == 0) {
    eventBootNonce = esp_random();
    if (eventBootNonce == 0) eventBootNonce = 1;
  }
  if (++eventSequence == 0) eventSequence++;

  PendingEventRecord &record = eventQueue[eventQueueIndex(eventQueueCount)];
  memset(&record, 0, sizeof(record));
  record.used = true;
  record.type = type;
  record.ignition = type == EV_ENGINE_ON ? 1 : 0;
  record.sequence = eventSequence;
  record.bootNonce = eventBootNonce;
  record.occurredMs = millis();
  if (lastValidPoint.valid) {
    record.position = lastValidPoint;
    record.fixAgeAtEventS = fixAgeS();
    record.positionSource = record.fixAgeAtEventS <= EVENT_FRESH_POSITION_MAX_AGE_S
                                ? EVENT_POSITION_FRESH : EVENT_POSITION_CACHE;
  } else {
    record.positionSource = EVENT_POSITION_NONE;
    record.fixAgeAtEventS = UINT32_MAX;
  }
  snprintf(record.eventId, sizeof(record.eventId), "%08lx-%08lx-%08lx",
           (unsigned long)(uint32_t)ESP.getEfuseMac(),
           (unsigned long)record.bootNonce, (unsigned long)record.sequence);
  eventQueueCount++;
  if (record.fixAgeAtEventS == UINT32_MAX) {
    SerialMon.printf("[EVENT] capturado id=%s type=%s pos=none cola=%u\n",
                     record.eventId, eventTypeName(type), (unsigned)eventQueueCount);
  } else {
    SerialMon.printf("[EVENT] capturado id=%s type=%s pos=%s age=%lus cola=%u\n",
                     record.eventId, eventTypeName(type),
                     eventPositionSourceName(record.positionSource),
                     (unsigned long)record.fixAgeAtEventS, (unsigned)eventQueueCount);
  }
  return true;
}

static void capturePendingTransition() {
  if (pendingEvent != EV_NONE && pendingEvent != lastCapturedPendingEvent &&
      enqueueIgnitionEvent(pendingEvent)) {
    lastCapturedPendingEvent = pendingEvent;
  }
}

static void popAcknowledgedEvents() {
  while (eventQueueCount > 0 && eventQueue[eventQueueHead].acknowledged) {
    SerialMon.printf("[EVENT] ACK id=%s type=%s\n",
                     eventQueue[eventQueueHead].eventId,
                     eventTypeName(eventQueue[eventQueueHead].type));
    memset(&eventQueue[eventQueueHead], 0, sizeof(PendingEventRecord));
    eventQueueHead = (uint8_t)((eventQueueHead + 1) % EVENT_QUEUE_CAPACITY);
    eventQueueCount--;
  }
  if (eventQueueCount == 0) {
    pendingEvent = EV_NONE;
    lastCapturedPendingEvent = EV_NONE;
  }
  lastEventAttemptMs = 0;
}

static void onIgnitionAck(char *topic, byte *payload, unsigned int length) {
  if (strcmp(topic, TOPIC_IGNITION_ACK_V3) != 0) return;
  char eventId[64];
  size_t n = length < sizeof(eventId) - 1 ? length : sizeof(eventId) - 1;
  memcpy(eventId, payload, n);
  eventId[n] = '\0';
  for (uint8_t offset = 0; offset < eventQueueCount; offset++) {
    PendingEventRecord &record = eventQueue[eventQueueIndex(offset)];
    if (record.used && strcmp(record.eventId, eventId) == 0) {
      record.acknowledged = true;
      popAcknowledgedEvents();
      return;
    }
  }
  SerialMon.printf("[EVENT] ACK desconocido id=%s\n", eventId);
}

static void ensureEventSubscription() {
  if (!mqtt.connected()) { eventMqttWasConnected = false; return; }
  if (eventMqttWasConnected) return;
  mqtt.setBufferSize(EVENT_MQTT_BUFFER_SIZE);
  mqtt.setCallback(onIgnitionAck);
  bool ok = mqtt.subscribe(TOPIC_IGNITION_ACK_V3, 1);
  SerialMon.printf("[EVENT] subscribe ack qos=1 result=%d\n", (int)ok);
  eventMqttWasConnected = true;
}

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

static void rememberPoint(const GpsPoint &point) {
  rtcFixAgeS = 0;
  fixAgeBaseMs = millis();
  lastValidPoint = point;
  if (point.speedValid && point.speed > MOVING_SPEED_KMH) lastMovementMs = millis();
}

static bool publishPointWithIgnition(const GpsPoint &point, const char *event, uint8_t ignition) {
  if (!mqtt.connected()) { SerialMon.println("[PUB] omitido: MQTT desconectado"); return false; }
  if (!point.valid) { SerialMon.println("[PUB] omitido: aún no hay posición válida en caché"); return false; }
  char speedField[16] = "", altField[16] = "";
  if (point.speedValid) snprintf(speedField, sizeof(speedField), "%.2f", point.speed);
  if (point.altValid) snprintf(altField, sizeof(altField), "%.1f", point.alt);
  char payload[256];
  snprintf(payload, sizeof(payload),
           "v2,%s,%u,%.6f,%.6f,%s,%s,%d,%.2f,%u,%s,%s",
           DEVICE_ID, point.fix, point.lat, point.lon, speedField, altField,
           point.vsat, point.acc, ignition, event, point.ts);
  bool ok = mqtt.publish(TOPIC_TELEMETRY, payload);
  SerialMon.printf("[PUB] %s topic=%s payload=%s\n", ok ? "OK" : "FAIL", TOPIC_TELEMETRY, payload);
  return ok;
}
static bool publishPoint(const GpsPoint &point, const char *event) {
  return publishPointWithIgnition(point, event, ignitionField());
}

static bool publishEventRecord(PendingEventRecord &record) {
  const char *type = eventTypeName(record.type);
  const char *state = record.ignition ? "on" : "off";
  const char *source = eventPositionSourceName(record.positionSource);
  char payload[640];
  if (record.positionSource == EVENT_POSITION_NONE) {
    snprintf(payload, sizeof(payload),
      "{\"v\":3,\"device\":\"%s\",\"event_id\":\"%s\",\"type\":\"%s\",\"ignition\":%u,\"occurred_ms\":%lu,\"position_source\":\"none\",\"fix_age_s\":null}",
      DEVICE_ID, record.eventId, type, record.ignition, (unsigned long)record.occurredMs);
  } else {
    snprintf(payload, sizeof(payload),
      "{\"v\":3,\"device\":\"%s\",\"event_id\":\"%s\",\"type\":\"%s\",\"ignition\":%u,\"occurred_ms\":%lu,\"position_source\":\"%s\",\"fix_age_s\":%lu,\"position\":{\"fix\":%u,\"lat\":%.6f,\"lon\":%.6f,\"speed\":%.2f,\"alt\":%.1f,\"sats\":%d,\"hdop\":%.2f,\"ts\":\"%s\"}}",
      DEVICE_ID, record.eventId, type, record.ignition,
      (unsigned long)record.occurredMs, source,
      (unsigned long)record.fixAgeAtEventS, record.position.fix,
      record.position.lat, record.position.lon, record.position.speed,
      record.position.alt, record.position.vsat, record.position.acc, record.position.ts);
  }
  bool stateOk = mqtt.publish(TOPIC_IGNITION_STATE_V3, state, true);
  bool legacyOk = mqtt.publish(TOPIC_IGNITION, state, true);
  bool eventOk = mqtt.publish(TOPIC_IGNITION_EVENT_V3, payload, false);
  record.attempts++;
  SerialMon.printf("[EVENT] intento=%u id=%s state=%d legacy=%d event=%d pos=%s\n",
                   (unsigned)record.attempts, record.eventId,
                   (int)stateOk, (int)legacyOk, (int)eventOk, source);
  return stateOk && eventOk;
}

static void serviceEvents() {
  capturePendingTransition();
  ensureEventSubscription();
  if (eventStoreEmpty() || !mqtt.connected()) return;
  uint32_t now = millis();
  if (lastEventAttemptMs && (now - lastEventAttemptMs) < EVENT_RETRY_MS) return;
  lastEventAttemptMs = now;
  publishEventRecord(eventQueue[eventQueueHead]);
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
  char buffer[16];
  snprintf(buffer, sizeof(buffer), "%.2f", readBatteryVolts());
  if (mqtt.publish(TOPIC_BATTERY, buffer, true)) {
    lastBatteryMs = now;
    batteryEverPublished = true;
  }
  SerialMon.printf("[BAT] %s V\n", buffer);
}
