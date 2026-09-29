#pragma once

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "espnow_protocol.h"
#include "espnow_secrets.h"

static bool espnowReady = false;
static uint16_t espnowSeq = 0;
static uint32_t espnowLastFastTelemMs = 0;
static uint32_t espnowLastHealthMs = 0;

static uint8_t getTrackerState() {
  if (millis() < 10000) {
    return ESPNOW_STATE_BOOT;
  }
  if (isIgnitionOff()) {
    return ESPNOW_STATE_PARKED;
  }
  bool moving = false;
  if (lastValidPoint.valid && lastValidPoint.speedValid && lastValidPoint.speed > MOVING_SPEED_KMH) {
    moving = true;
  } else if ((millis() - lastMovementMs) <= MOVING_HOLD_MS) {
    moving = true;
  }
  
  if (moving) {
    return ESPNOW_STATE_MOVING;
  } else {
    return ESPNOW_STATE_IDLE;
  }
}

static void espnowInit() {
  SerialMon.println("[ESPNOW] Inicializando...");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) {
    SerialMon.println("[ESPNOW] Fallo al inicializar esp_now");
    espnowReady = false;
    return;
  }

  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, ESPNOW_PEER_MAC, 6);
  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    SerialMon.println("[ESPNOW] Fallo al añadir el peer");
    espnowReady = false;
    return;
  }

  espnowReady = true;
  SerialMon.println("[ESPNOW] Listo");
}

static void espnowSuspend() {
  if (!espnowReady) return;
  SerialMon.println("[ESPNOW] Suspendiendo...");
  esp_now_deinit();
  WiFi.mode(WIFI_OFF);
  espnowReady = false;
}

static void espnowSendEvent(uint8_t eventType) {
  if (!espnowReady) return;
  EspNowEvent evt;
  memset(&evt, 0, sizeof(evt));
  
  evt.hdr.version = ESPNOW_PROTOCOL_VERSION;
  evt.hdr.msgType = ESPNOW_MSG_EVENT;
  evt.hdr.sequence = espnowSeq++;
  evt.hdr.length = sizeof(EspNowEvent);
  evt.hdr.flags = 0;
  
  uint32_t age = fixAgeS() * 10;
  if (age > 254) age = ESPNOW_AGE_UNKNOWN;
  evt.hdr.ageDs = (uint8_t)age;
  
  evt.eventType = eventType;
  evt.ignState = ignState;
  evt.batMv = (uint16_t)(readBatteryVolts() * 1000.0f);
  evt.vbusMv = (uint16_t)(ignLastPinV * DIVIDER_FACTOR * 1000.0f);
  evt.uptimeS = (uint16_t)(millis() / 1000);
  
  espnowSignPacket(&evt, sizeof(evt));
  esp_now_send(ESPNOW_PEER_MAC, (const uint8_t *)&evt, sizeof(evt));
}

static void sendFastTelem() {
  EspNowFastTelem pkt;
  memset(&pkt, 0, sizeof(pkt));
  
  pkt.hdr.version = ESPNOW_PROTOCOL_VERSION;
  pkt.hdr.msgType = ESPNOW_MSG_FAST_TELEM;
  pkt.hdr.sequence = espnowSeq++;
  pkt.hdr.length = sizeof(EspNowFastTelem);
  pkt.hdr.flags = 0;
  
  uint32_t age = fixAgeS() * 10;
  if (age > 254) age = ESPNOW_AGE_UNKNOWN;
  pkt.hdr.ageDs = (uint8_t)age;
  
  if (lastValidPoint.valid) {
    if (lastValidPoint.speedValid) {
      pkt.speedX10 = (int16_t)(lastValidPoint.speed * 10.0f);
      pkt.hdr.flags |= ESPNOW_VALID_SPEED;
    }
    pkt.heading = 0;
    if (lastValidPoint.altValid) {
      pkt.altitude = (int16_t)lastValidPoint.alt;
      pkt.hdr.flags |= ESPNOW_VALID_ALTITUDE;
    }
    pkt.hdr.flags |= ESPNOW_VALID_GNSS_FIX;
  }
  
  pkt.rpm = ESPNOW_RPM_UNAVAILABLE;
  pkt.speedSource = ESPNOW_SOURCE_GPS;
  
  espnowSignPacket(&pkt, sizeof(pkt));
  esp_now_send(ESPNOW_PEER_MAC, (const uint8_t *)&pkt, sizeof(pkt));
}

static void sendHealth() {
  EspNowHealth pkt;
  memset(&pkt, 0, sizeof(pkt));
  
  pkt.hdr.version = ESPNOW_PROTOCOL_VERSION;
  pkt.hdr.msgType = ESPNOW_MSG_HEALTH;
  pkt.hdr.sequence = espnowSeq++;
  pkt.hdr.length = sizeof(EspNowHealth);
  pkt.hdr.flags = 0;
  
  uint32_t age = fixAgeS() * 10;
  if (age > 254) age = ESPNOW_AGE_UNKNOWN;
  pkt.hdr.ageDs = (uint8_t)age;
  
  pkt.trackerState = getTrackerState();
  if (lastValidPoint.valid) {
    pkt.gnssFix = lastValidPoint.fix;
    pkt.gnssSats = lastValidPoint.vsat;
    pkt.gnssHdopX10 = (uint8_t)(lastValidPoint.acc * 10.0f);
  }
  
  pkt.lteSignal = (uint8_t)modem.getSignalQuality();
  pkt.lteRegistered = modem.isGprsConnected() ? 1 : 0;
  pkt.mqttConnected = mqtt.connected() ? 1 : 0;
  pkt.mqttFailCount = mqttFailCount;
  
  pkt.batMv = (uint16_t)(readBatteryVolts() * 1000.0f);
  pkt.vbusMv = (uint16_t)(readPinVolts() * DIVIDER_FACTOR * 1000.0f);
  
  pkt.hdr.flags |= ESPNOW_VALID_BATTERY | ESPNOW_VALID_VOLTAGE;
  
  pkt.ignState = ignState;
  pkt.wakeReason = wakeCause;
  pkt.bootCount = (uint16_t)rtcBootCount;
  
  uint32_t fa = fixAgeS();
  pkt.fixAgeS = fa > 65534 ? 65535 : (uint16_t)fa;
  pkt.gnssStale = (uint16_t)rtcGnssStale;
  
  pkt.fwMajor = 1;
  pkt.fwMinor = 0;
  
  espnowSignPacket(&pkt, sizeof(pkt));
  esp_now_send(ESPNOW_PEER_MAC, (const uint8_t *)&pkt, sizeof(pkt));
}

static void espnowService() {
  if (!espnowReady) return;
  if (isIgnitionOff()) return;

  uint32_t now = millis();
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
