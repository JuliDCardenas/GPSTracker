#pragma once
// Motor GNSS de producción y huella verificable del firmware.

#define GNSS_PWR_TIMEOUT_MS 10000UL
#define GNSS_SETTLE_MS 2000UL
#define GNSS_PWR_ATTEMPTS 2
#define GNSS_FORCE_MODE 0
#define GNSS_STALE_LOG_MS 60000UL

#define FW_NAME "tracker"
#define FW_VERSION "2.3.0-rc1"
#ifndef TRACKER_GIT_SHA
#define TRACKER_GIT_SHA "unknown"
#endif
static const char FW_BUILD[] = TRACKER_GIT_SHA "," __DATE__ " " __TIME__;
static const char TOPIC_FW[] = "tracker/Lilygo/sys/fw";
static const char TOPIC_GNSS[] = "tracker/Lilygo/sys/gnss";

static RTC_DATA_ATTR uint16_t rtcGnssStale = 0;
static uint32_t gnssOnAtMs = 0;
static bool gnssTtffPending = false;

static bool gnssPwrOn() {
  modem.sendAT("+CGDRT=", MODEM_GPS_ENABLE_GPIO, ",1");
  modem.waitResponse(2000);
  modem.sendAT("+CGSETV=", MODEM_GPS_ENABLE_GPIO, ",", MODEM_GPS_ENABLE_LEVEL);
  modem.waitResponse(2000);

  bool ok = false;
  for (uint8_t i = 0; i < GNSS_PWR_ATTEMPTS && !ok; i++) {
    modem.sendAT("+CGNSSPWR=1");
    ok = (modem.waitResponse(GNSS_PWR_TIMEOUT_MS) == 1);
    if (!ok) {
      SerialMon.printf("[GNSS] AT+CGNSSPWR=1 sin OK (intento %u de %u)\n",
                       (unsigned)(i + 1), (unsigned)GNSS_PWR_ATTEMPTS);
      watchdogFeed();
      delay(500);
    }
  }
  if (!ok) {
    SerialMon.println("[GNSS] el modem no confirmo el power -> se sigue sin GNSS");
    return false;
  }

  delay(GNSS_SETTLE_MS);
  gnssOnAtMs = millis();
  gnssTtffPending = true;
#if GNSS_FORCE_MODE
  modem.setGPSBaud(115200);
  if (modem.setGPSMode(15)) SerialMon.println("[GNSS] modo 15 escrito");
  else SerialMon.println("[GNSS] modo 15 FAIL; sigue el modo guardado");
#else
  modem.sendAT("+CGNSSMODE?");
  modem.waitResponse(2000);
#endif
  SerialMon.printf("[GNSS] motor encendido en ~%lu ms\n", (unsigned long)GNSS_SETTLE_MS);
  return true;
}

static bool gnssFrameIsStale(const char *ts, bool lastValid, const char *lastTs) {
  if (!lastValid || strcmp(ts, lastTs) != 0) return false;
  rtcGnssStale++;
  static uint32_t lastStaleLogMs = 0;
  uint32_t now = millis();
  if (lastStaleLogMs == 0 || (now - lastStaleLogMs) > GNSS_STALE_LOG_MS) {
    lastStaleLogMs = now;
    SerialMon.printf("[GPS] trama rancia: %s repetida -> descartada (total=%u)\n",
                     ts, (unsigned)rtcGnssStale);
  }
  return true;
}

static void gnssNoteFirstFix(int sats, float hdop) {
  if (!gnssTtffPending) return;
  gnssTtffPending = false;
  SerialMon.printf("[GNSS] TTFF real %lus (sats=%d hdop=%.2f)\n",
                   (unsigned long)((millis() - gnssOnAtMs) / 1000), sats, hdop);
}

static void gnssLogSpeedUnits(float speed) {
  if (speed <= 0.5f) return;
  SerialMon.printf("[GPS] velocidad cruda=%.2f (nudos -> %.1f km/h)\n",
                   speed, speed * 1.852f);
}

static void gnssFwString(char *buf, size_t size) {
  snprintf(buf, size, "%s %s,sha=%s,build=%s %s",
           FW_NAME, FW_VERSION, TRACKER_GIT_SHA, __DATE__, __TIME__);
}
