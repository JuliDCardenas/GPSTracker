#pragma once
// Reintentos acotados del evento pendiente durante parqueo.

#define PARKED_EVENT_RETRY_WINDOW_MS 15000UL

static RTC_DATA_ATTR uint8_t rtcParkedEventRetryStage = 0;
static RTC_DATA_ATTR char rtcParkedRetryEventId[48] = {};

static uint32_t parkedRetryTargetS(uint8_t stage) {
  if (stage == 0) return 2UL * 60UL;
  if (stage == 1) return 10UL * 60UL;
  if (stage == 2) return 30UL * 60UL;
  return PARKED_PULSE_S;
}

static void resetParkedRetryIfHeadChanged() {
  if (eventStoreEmpty()) {
    rtcParkedEventRetryStage = 0;
    rtcParkedRetryEventId[0] = '\0';
    return;
  }
  const char *headId = eventQueue[eventQueueHead].eventId;
  if (strcmp(rtcParkedRetryEventId, headId) != 0) {
    snprintf(rtcParkedRetryEventId, sizeof(rtcParkedRetryEventId), "%s", headId);
    rtcParkedEventRetryStage = 0;
  }
}

static void parkedRetryPendingEvents() {
  resetParkedRetryIfHeadChanged();
  if (eventStoreEmpty() || rtcParkedEventRetryStage >= 3) return;

  uint32_t elapsedAfterThisTick = rtcSleptSeconds + PARKED_POLL_S;
  uint32_t target = parkedRetryTargetS(rtcParkedEventRetryStage);
  if (elapsedAfterThisTick < target) return;

  SerialMon.printf("[EVENT] retry parqueado etapa=%u t=%lus id=%s\n",
                   (unsigned)(rtcParkedEventRetryStage + 1),
                   (unsigned long)elapsedAfterThisTick,
                   eventQueue[eventQueueHead].eventId);
  rtcParkedEventRetryStage++;

  pmModemResume();
  ensureLTE();
  tryConnectMQTT();

  uint32_t started = millis();
  while (!eventStoreEmpty() && (millis() - started) < PARKED_EVENT_RETRY_WINDOW_MS) {
    serviceMQTT();
    mqtt.loop();
    serviceEvents();
    watchdogFeed();
    delay(25);
  }

  if (eventStoreEmpty()) {
    SerialMon.println("[EVENT] retry parqueado confirmado por ACK");
    rtcParkedEventRetryStage = 0;
    rtcParkedRetryEventId[0] = '\0';
  } else {
    SerialMon.printf("[EVENT] retry parqueado sin ACK; próxima etapa=%u\n",
                     (unsigned)rtcParkedEventRetryStage);
  }

  pmDisconnectClean();
  pmModemSleep();
}
