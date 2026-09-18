#pragma once
// Validación del despertar, forense y reintentos estacionados.

#include "tracker_parked_retry.h"

#define TOPIC_WAKE "tracker/Lilygo/sys/wake"

static void wakeServiceTimerTick(float bootPinV) {
#if !TEST_DISABLE_SLEEP
  if (wakeCause != ESP_SLEEP_WAKEUP_TIMER) return;
#if TEST_FORCE_PARKED
  bool sigueParqueado = true;
#else
  bool sigueParqueado = (bootPinV < PIN_ON_V);
#endif
  if (sigueParqueado) {
    parkedRetryPendingEvents();
    pmParkedTick();
  }
  SerialMon.printf("[PM] repaso con pin ARRIBA (%.3fV >= %.2fV) -> sigue arranque\n",
                   bootPinV, PIN_ON_V);
#else
  (void)bootPinV;
#endif
}

static void wakeConfirmExt0(bool coldBoot) {
#if !TEST_DISABLE_SLEEP && !TEST_FORCE_PARKED
  if (coldBoot || wakeCause != ESP_SLEEP_WAKEUP_EXT0) return;
  uint32_t started = millis();
  bool sustained = false;
  while ((millis() - started) < ON_DEBOUNCE_MS) {
    if (readPinVolts() < PIN_ON_V) { sustained = false; break; }
    sustained = true;
    watchdogFeed();
    delay(IGN_SAMPLE_MS);
  }
  if (!sustained) {
    rtcSpuriousExt0++;
    SerialMon.printf("[PM] ext0 espurio #%u: pin=%.3fV no sostuvo %lums\n",
                     (unsigned)rtcSpuriousExt0, readPinVolts(),
                     (unsigned long)ON_DEBOUNCE_MS);
    ignState = IGN_OFF;
    pmDeepSleep(true, PARKED_POLL_S);
  }
  SerialMon.printf("[PM] ext0 confirmado durante %lums\n",
                   (unsigned long)ON_DEBOUNCE_MS);
#else
  (void)coldBoot;
#endif
}

static void wakePublishForensics(float bootPinV) {
  if (!mqtt.connected()) return;
  char wake[96];
  snprintf(wake, sizeof(wake), "cause=%d,pin=%.3f,boot=%lu,spur=%u,fixage=%lu",
           (int)wakeCause, bootPinV, (unsigned long)rtcBootCount,
           (unsigned)rtcSpuriousExt0, (unsigned long)fixAgeS());
  mqtt.publish(TOPIC_WAKE, wake, true);
  SerialMon.printf("[PM] wake %s\n", wake);
}
