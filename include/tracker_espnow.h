#pragma once

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "espnow_protocol.h"
#include "espnow_secrets.h"

// Estados de hardware del radio
static bool wifiRadioOn = false;
static bool espnowReady = false;

// Secuencia e intervalos
static uint16_t espnowSeq = 0;
static uint32_t espnowLastFastTelemMs = 0;
static uint32_t espnowLastHealthMs = 0;

// Caché de estado LTE para no saturar el módem con comandos AT a 1 Hz (Defecto 4)
static uint8_t cachedLteSignal = 99;      // 99 = desconocido
static uint8_t cachedLteRegistered = 0;   // 0 = no registrado
static uint32_t lastLteCheckMs = 0;
static const uint32_t LTE_CHECK_INTERVAL_MS = 25000; // Consulta cada 25 segundos

// Observabilidad y estadísticas de envío (Defecto 7)
static uint32_t espnowPacketsAttempted = 0;
static uint32_t espnowPacketsAcceptedApi = 0;
static uint32_t espnowPacketsCbSuccess = 0;
static uint32_t espnowPacketsCbFail = 0;

static void onEspNowDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    espnowPacketsCbSuccess++;
  } else {
    espnowPacketsCbFail++;
  }
}

// Envío seguro y medición de retorno inmediato de la API
static bool espnowSendRaw(const uint8_t *mac, const uint8_t *data, size_t len) {
  espnowPacketsAttempted++;
  esp_err_t err = esp_now_send(mac, data, len);
  if (err == ESP_OK) {
    espnowPacketsAcceptedApi++;
    return true;
  }
  return false;
}

// Conversión de velocidad GNSS (el módem SIM7670G entrega nudos en AT+CGNSSINFO) a km/h (Defecto 8)
static inline float gnssKnotsToKmh(float speedKnots) {
  return speedKnots * 1.852f;
}

// Evaluación del estado del tracker con velocidad normalizada a km/h
static uint8_t getTrackerState() {
  if (millis() < 10000) {
    return ESPNOW_STATE_BOOT;
  }
  if (isIgnitionOff()) {
    return ESPNOW_STATE_PARKED;
  }

  bool moving = false;
  if (lastValidPoint.valid && lastValidPoint.speedValid) {
    float speedKmh = gnssKnotsToKmh(lastValidPoint.speed);
    if (speedKmh > MOVING_SPEED_KMH) {
      moving = true;
    }
  }

  if (!moving && ((millis() - lastMovementMs) <= MOVING_HOLD_MS)) {
    moving = true;
  }

  return moving ? ESPNOW_STATE_MOVING : ESPNOW_STATE_IDLE;
}

// Suspensión garantizada de Wi-Fi y ESP-NOW en cualquier circunstancia (Defecto 5)
static void espnowSuspend() {
  if (espnowReady) {
    esp_now_deinit();
    espnowReady = false;
  }
  if (wifiRadioOn) {
    WiFi.mode(WIFI_OFF);
    wifiRadioOn = false;
  }
  SerialMon.println("[ESPNOW] Suspendido: WiFi apagado");
}

// Inicialización de radio y ESP-NOW con limpieza garantizada ante fallos (Defecto 5)
static void espnowInit() {
  SerialMon.println("[ESPNOW] Inicializando...");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  wifiRadioOn = true;

  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) {
    SerialMon.println("[ESPNOW] Fallo al inicializar esp_now -> suspendiendo WiFi");
    espnowSuspend();
    return;
  }

  // Registrar callback de confirmación para observabilidad (Defecto 7)
  esp_now_register_send_cb(onEspNowDataSent);

  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, ESPNOW_PEER_MAC, 6);
  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    SerialMon.println("[ESPNOW] Fallo al añadir peer -> suspendiendo WiFi");
    espnowSuspend();
    return;
  }

  espnowReady = true;
  SerialMon.printf("[ESPNOW] Listo en canal %d\n", ESPNOW_CHANNEL);
}

// Envío inmediato de eventos críticos (Defecto 1 y Defecto 7)
static void espnowSendEvent(uint8_t eventType) {
  if (!espnowReady) return;

  EspNowEvent evt;
  memset(&evt, 0, sizeof(evt));

  evt.hdr.version = ESPNOW_PROTOCOL_VERSION;
  evt.hdr.msgType = ESPNOW_MSG_EVENT;
  evt.hdr.sequence = espnowSeq++;
  evt.hdr.length = sizeof(EspNowEvent);
  evt.hdr.flags = 0;

  if (lastValidPoint.valid) {
    uint32_t ageS = fixAgeS();
    uint32_t ageDs = ageS * 10;
    evt.hdr.ageDs = (ageDs > 254) ? 254 : (uint8_t)ageDs;
  } else {
    evt.hdr.ageDs = ESPNOW_AGE_UNKNOWN;
  }

  evt.eventType = eventType;
  evt.ignState = ignState;
  evt.batMv = (uint16_t)(readBatteryVolts() * 1000.0f);
  evt.vbusMv = (uint16_t)(ignLastPinV * DIVIDER_FACTOR * 1000.0f);
  evt.uptimeS = (uint16_t)(millis() / 1000);

  espnowSignPacket(&evt, sizeof(evt));
  espnowSendRaw(ESPNOW_PEER_MAC, (const uint8_t *)&evt, sizeof(evt));
}

// Envío de Telemetría Rápida (Defecto 1 y Defecto 8)
static void sendFastTelem() {
  EspNowFastTelem pkt;
  memset(&pkt, 0, sizeof(pkt));

  pkt.hdr.version = ESPNOW_PROTOCOL_VERSION;
  pkt.hdr.msgType = ESPNOW_MSG_FAST_TELEM;
  pkt.hdr.sequence = espnowSeq++;
  pkt.hdr.length = sizeof(EspNowFastTelem);
  pkt.hdr.flags = 0;

  if (lastValidPoint.valid) {
    uint32_t ageS = fixAgeS();
    uint32_t ageDs = ageS * 10;
    // Saturar en 254 si supera 25.4s; 255 reservado estrictamente para ESPNOW_AGE_UNKNOWN
    pkt.hdr.ageDs = (ageDs > 254) ? 254 : (uint8_t)ageDs;
    pkt.hdr.flags |= ESPNOW_VALID_GNSS_FIX;

    if (lastValidPoint.speedValid) {
      float speedKmh = gnssKnotsToKmh(lastValidPoint.speed);
      pkt.speedX10 = (int16_t)(speedKmh * 10.0f);
      pkt.hdr.flags |= ESPNOW_VALID_SPEED;
      pkt.speedSource = ESPNOW_SOURCE_GPS;
    } else {
      pkt.speedX10 = 0;
      pkt.speedSource = ESPNOW_SOURCE_NONE;
    }

    if (lastValidPoint.altValid) {
      pkt.altitude = (int16_t)lastValidPoint.alt;
      pkt.hdr.flags |= ESPNOW_VALID_ALTITUDE;
    } else {
      pkt.altitude = 0;
    }

    pkt.heading = 0; // Sin sensor de brújula adicional por ahora
  } else {
    // Si no hay posición válida, indicar edad desconocida y no activar banderas (Defecto 1)
    pkt.hdr.ageDs = ESPNOW_AGE_UNKNOWN;
    pkt.speedX10 = 0;
    pkt.altitude = 0;
    pkt.heading = 0;
    pkt.speedSource = ESPNOW_SOURCE_NONE;
  }

  pkt.rpm = ESPNOW_RPM_UNAVAILABLE; // Futura integración OBD-II

  espnowSignPacket(&pkt, sizeof(pkt));
  espnowSendRaw(ESPNOW_PEER_MAC, (const uint8_t *)&pkt, sizeof(pkt));
}

// Envío de Salud y Estado del Tracker (Defecto 4 y Defecto 6)
static void sendHealth() {
  EspNowHealth pkt;
  memset(&pkt, 0, sizeof(pkt));

  pkt.hdr.version = ESPNOW_PROTOCOL_VERSION;
  pkt.hdr.msgType = ESPNOW_MSG_HEALTH;
  pkt.hdr.sequence = espnowSeq++;
  pkt.hdr.length = sizeof(EspNowHealth);
  pkt.hdr.flags = 0;

  if (lastValidPoint.valid) {
    uint32_t ageS = fixAgeS();
    uint32_t ageDs = ageS * 10;
    pkt.hdr.ageDs = (ageDs > 254) ? 254 : (uint8_t)ageDs;
    pkt.gnssFix = lastValidPoint.fix;
    pkt.gnssSats = lastValidPoint.vsat;
    pkt.gnssHdopX10 = (uint8_t)(lastValidPoint.acc * 10.0f);
    pkt.hdr.flags |= ESPNOW_VALID_GNSS_FIX;
  } else {
    pkt.hdr.ageDs = ESPNOW_AGE_UNKNOWN;
    pkt.gnssFix = 0;
    pkt.gnssSats = 0;
    pkt.gnssHdopX10 = 0;
  }

  // Asignación explícita del estado operativo del tracker (Hallazgo 1)
  pkt.trackerState = getTrackerState();

  // Uso de caché para no enviar comandos AT en el hilo de 1 Hz (Defecto 4 / Hallazgo 3)
  pkt.lteSignal = cachedLteSignal;
  pkt.lteRegistered = cachedLteRegistered;
  pkt.mqttConnected = mqtt.connected() ? 1 : 0;
  pkt.mqttFailCount = mqttFailCount;

  pkt.batMv = (uint16_t)(readBatteryVolts() * 1000.0f);
  pkt.vbusMv = (uint16_t)(readPinVolts() * DIVIDER_FACTOR * 1000.0f);
  pkt.hdr.flags |= ESPNOW_VALID_BATTERY | ESPNOW_VALID_VOLTAGE;

  pkt.ignState = ignState;
  pkt.wakeReason = wakeCause;
  pkt.bootCount = (uint16_t)rtcBootCount;

  uint32_t fa = fixAgeS();
  pkt.fixAgeS = (fa > 65534) ? 65535 : (uint16_t)fa;
  pkt.gnssStale = (uint16_t)rtcGnssStale;

  // Versión oficial desacoplada desde gnss_prod.h (Defecto 6)
  pkt.fwMajor = FW_VERSION_MAJOR;
  pkt.fwMinor = FW_VERSION_MINOR;

  espnowSignPacket(&pkt, sizeof(pkt));
  espnowSendRaw(ESPNOW_PEER_MAC, (const uint8_t *)&pkt, sizeof(pkt));
}

// Actualización explícita de métricas LTE: solo se llama durante setup o reconexión de red,
// NUNCA en el superloop rápido de 1 Hz (Hallazgo 3)
static void espnowUpdateLteStatus() {
  int csq = modem.getSignalQuality();
  if (csq >= 0 && csq <= 31) {
    cachedLteSignal = (uint8_t)csq;
  }
  cachedLteRegistered = modem.isGprsConnected() ? 1 : 0;
}

// Servicio periódico llamado desde loop() - 100% libre de comandos AT
static void espnowService() {
  if (!espnowReady) return;
  if (isIgnitionOff()) return;

  uint32_t now = millis();

  // Si MQTT está conectado, sabemos que GPRS está registrado sin enviar comandos AT
  if (mqtt.connected()) {
    cachedLteRegistered = 1;
  }

  uint8_t state = getTrackerState();
  uint32_t fastCadenceMs = (state == ESPNOW_STATE_MOVING) ? 500 : 1000;

  if (now - espnowLastFastTelemMs >= fastCadenceMs) {
    sendFastTelem();
    espnowLastFastTelemMs = now;
  }

  if (now - espnowLastHealthMs >= 1000) {
    sendHealth();
    espnowLastHealthMs = now;
  }
}
