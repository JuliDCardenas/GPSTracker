#pragma once
//
// tracker_telemetry.h — lectura del GNSS y publicacion
// Proyecto GPS Tracker Logan · rama feat/low-power-nivel2
//
// ESTE ARCHIVO NO ES INDEPENDIENTE. Salio de main.cpp y se incluye UNA sola vez,
// desde main.cpp, justo despues de serviceMQTT(). En ese punto ya existen y por
// eso aqui se usan sin declararlos: modem, mqtt, SerialMon, GpsPoint,
// lastValidPoint, lastMovementMs, lastPublishMs, lastBatteryMs,
// batteryEverPublished, isGpsPositionValid(), isGpsTimeValid(),
// buildGpsQuality(), ignitionField(), isIgnitionOff(), currentPeriodMs(),
// readBatteryVolts(), los TOPIC_* y las funciones de gnss_prod.h.
//

// ---------- Telemetria ----------
// Lee el GNSS y, si el punto es valido, lo deja en out. Es bloqueante (AT+CGNSSINFO
// tarda tipicamente 50-300 ms), por eso solo se llama en los instantes de
// publicacion y no en cada vuelta del loop.
static bool readGpsPoint(GpsPoint &out) {
  float lat = 0, lon = 0, speed = 0, alt = 0, acc = 0;
  int vsat = 0, usat = 0;
  int year = 0, month = 0, day = 0, hour = 0, min = 0, sec = 0;
  uint8_t fix = 0;

  bool ok = modem.getGPS(&fix, &lat, &lon, &speed, &alt, &vsat, &usat, &acc,
                         &year, &month, &day, &hour, &min, &sec);

  if (!ok) {
    SerialMon.println("[GPS] read FAIL");
    return false;
  }

  if (!isGpsPositionValid(fix, lat, lon, vsat, acc)) {
    SerialMon.printf("[GPS] posición inválida fix=%u lat=%.6f lon=%.6f speed=%.2f alt=%.1f sats=%d hdop=%.2f\n",
                     fix, lat, lon, speed, alt, vsat, acc);
    return false;
  }

  // Canario de parseo corrupto: si la fecha no existe en este planeta, el resto
  // de la trama tampoco es confiable aunque la lat/lon se vea razonable.
  if (!isGpsTimeValid(year, month, day, hour, min, sec)) {
    SerialMon.printf("[GPS] trama corrupta, fecha %04d-%02d-%02dT%02d:%02d:%02d -> descartada\n",
                     year, month, day, hour, min, sec);
    return false;
  }

  // F22 (2026-08-22) — GUARDIA DE TRAMA RANCIA. Ver la SEPTIMA REGLA.
  //
  // Hasta aqui la trama paso el filtro de calidad y el canario de fecha, y sin
  // embargo puede ser la ULTIMA SOLUCION GUARDADA que el modem recicla como si
  // fuera nueva. En el banco de hoy la trama vieja traia MEJORES sats y HDOP que
  // las reales, asi que ningun filtro de calidad la iba a atrapar: lo unico que
  // la delata es que la marca de tiempo no avanzo.
  //
  // Va DESPUES del canario de fecha a proposito: si la fecha viene corrupta, la
  // marca no sirve ni para comparar.
  char tsNow[24];
  snprintf(tsNow, sizeof(tsNow), "%04d-%02d-%02dT%02d:%02d:%02dZ",
           year, month, day, hour, min, sec);

  if (gnssFrameIsStale(tsNow, lastValidPoint.valid, lastValidPoint.ts)) {
    // El contador vive en RTC; la publicacion es oportunista y va limitada por
    // GNSS_STALE_LOG_MS dentro de gnssFrameIsStale(), que ya decidio si toca
    // reportar. Aqui solo se acompana con el topic de sistema.
    if (mqtt.connected()) {
      char g[64];
      snprintf(g, sizeof(g), "stale=%u,last=%s", (unsigned)rtcGnssStale, tsNow);
      mqtt.publish(TOPIC_GNSS, g, true);
    }
    return false;
  }

  // Primer fix FRESCO tras encender el motor GNSS: unico TTFF de produccion que
  // vamos a tener. Todos los demas numeros de TTFF del proyecto son de banco.
  gnssNoteFirstFix(vsat, acc);

  // F20: solo instrumentacion, el CSV no se toca. Ver gnssLogSpeedUnits().
  gnssLogSpeedUnits(speed);

  uint8_t quality = buildGpsQuality(speed, alt);

  out.valid = true;
  out.fix = fix;
  out.lat = lat;
  out.lon = lon;
  out.speed = speed;
  out.alt = alt;
  out.acc = acc;
  out.vsat = vsat;
  out.speedValid = (quality & GPS_QUALITY_SPEED_VALID) != 0;
  out.altValid = (quality & GPS_QUALITY_ALT_VALID) != 0;
  snprintf(out.ts, sizeof(out.ts), "%04d-%02d-%02dT%02d:%02d:%02dZ",
           year, month, day, hour, min, sec);

  if (!out.speedValid || !out.altValid) {
    SerialMon.printf("[GPS] posición válida con datos parciales quality=%u speed=%.2f alt=%.1f sats=%d hdop=%.2f\n",
                     quality, speed, alt, vsat, acc);
  }

  return true;
}

// Actualiza la cache y la marca de movimiento con un punto fresco.
static void rememberPoint(const GpsPoint &p) {
  lastValidPoint = p;
  if (p.speedValid && p.speed > MOVING_SPEED_KMH) {
    lastMovementMs = millis();
  }
}

static bool publishPoint(const GpsPoint &p, const char *event) {
  if (!mqtt.connected()) {
    SerialMon.println("[PUB] omitido: MQTT desconectado");
    return false;
  }
  if (!p.valid) {
    SerialMon.println("[PUB] omitido: aún no hay posición válida en caché");
    return false;
  }

  char speedField[16] = "";
  char altField[16] = "";

  if (p.speedValid) {
    snprintf(speedField, sizeof(speedField), "%.2f", p.speed);
  }
  if (p.altValid) {
    snprintf(altField, sizeof(altField), "%.1f", p.alt);
  }

  char payload[256];
  snprintf(payload, sizeof(payload),
           "v2,%s,%u,%.6f,%.6f,%s,%s,%d,%.2f,%u,%s,%s",
           DEVICE_ID, p.fix, p.lat, p.lon, speedField, altField, p.vsat, p.acc,
           ignitionField(), event, p.ts);

  // Telemetría GPS en QoS 0: alto volumen, perder un punto no es crítico.
  bool pubOk = mqtt.publish(TOPIC_TELEMETRY, payload);
  SerialMon.printf("[PUB] %s topic=%s payload=%s\n", pubOk ? "OK" : "FAIL", TOPIC_TELEMETRY, payload);
  return pubOk;
}

// Publica los eventos de transicion apenas haya enlace. Si MQTT esta caido, el
// evento queda pendiente y sale en cuanto vuelva: no se pierde el apagado.
static void serviceEvents() {
  if (pendingEvent == EV_NONE) {
    return;
  }
  if (!mqtt.connected()) {
    return;
  }

  bool isOn = (pendingEvent == EV_ENGINE_ON);
  const char *eventName = isOn ? EVENT_ENGINE_ON : EVENT_ENGINE_OFF;

  // Estado retenido para n8n y depuracion.
  mqtt.publish(TOPIC_IGNITION, isOn ? "on" : "off", true);

  if (isOn) {
    // Arranca el reloj del ralenti: tras encender, el vehiculo tiene garantizada
    // la cadencia rapida durante MOVING_HOLD_MS aunque todavia no se mueva.
    lastMovementMs = millis();
  }

  // Al encender vale la pena intentar un punto fresco. Al apagar tambien se
  // intenta, pero si el GNSS no responde se publica la ultima posicion valida:
  // ese es justamente el "aqui quedo parqueado".
  GpsPoint fresh = {};
  if (readGpsPoint(fresh)) {
    rememberPoint(fresh);
  }

  publishPoint(lastValidPoint, eventName);
  lastPublishMs = millis();
  pendingEvent = EV_NONE;
}

static void serviceTelemetry() {
  uint32_t now = millis();
  if ((now - lastPublishMs) < currentPeriodMs()) {
    return;
  }
  lastPublishMs = now;

  GpsPoint fresh = {};
  bool haveFresh = readGpsPoint(fresh);
  if (haveFresh) {
    rememberPoint(fresh);
  }

  if (haveFresh) {
    publishPoint(lastValidPoint, EVENT_NONE);
    return;
  }

  // Sin fix fresco: con el motor encendido no se publica (igual que antes,
  // se reintenta en el siguiente ciclo). Pero el keepalive de parqueo si sale
  // con la ultima posicion conocida, porque su funcion es avisar "sigo aqui".
  if (isIgnitionOff()) {
    publishPoint(lastValidPoint, EVENT_NONE);
  }
}

static void serviceBattery() {
  // Parqueado, la bateria viaja al ritmo del keepalive y no cada 5 min.
  uint32_t period = isIgnitionOff() ? (PARKED_KEEPALIVE_MIN * 60000UL) : BATTERY_PERIOD_MS;

  uint32_t now = millis();
  // La primera lectura sale apenas hay enlace. Sin esto, con el motor encendido
  // nos quedabamos sin dato de bateria los primeros 5 minutos (banco 2026-08-18).
  if (batteryEverPublished && (now - lastBatteryMs) < period) {
    return;
  }
  if (!mqtt.connected()) {
    return;
  }

  float volts = readBatteryVolts();
  char buf[16];
  snprintf(buf, sizeof(buf), "%.2f", volts);
  mqtt.publish(TOPIC_BATTERY, buf, true);
  lastBatteryMs = now;
  batteryEverPublished = true;
  SerialMon.printf("[BAT] %s V\n", buf);
}
