#pragma once
//
// tracker_pm.h — Nivel 2 de ahorro de energia y corte por bajo voltaje
// Proyecto GPS Tracker Logan · rama feat/low-power-nivel2
//
// ESTE ARCHIVO NO ES INDEPENDIENTE. Salio de main.cpp sin cambios de logica y
// se incluye UNA sola vez, desde main.cpp, justo antes de setup(). En ese punto
// ya existen y por eso aqui se usan sin declararlos: modem, netClient, mqtt,
// SerialMon, wakeCause, lastValidPoint, ignState, pendingEvent, readPinVolts(),
// readBatteryVolts(), batteryReadingPlausible(), publishStatus(),
// publishPoint(), serviceEvents(), tryConnectMQTT(), ensureLTE(), waitForAT(),
// modemPowerOn(), watchdogFeed() y gnssPwrOn() de gnss_prod.h.
//
// Se partio porque main.cpp llego a 67 KB y ya no se podia reescribir de una
// sola pasada con herramientas remotas: el 2026-08-22 un intento dejo el
// archivo truncado a mitad de tryConnectMQTT() y el repo sin compilar.
//

// ===================== NIVEL 2 + CORTE POR BAJO VOLTAJE =====================
// Estado que sobrevive al deep sleep (RTC slow memory).
static RTC_DATA_ATTR uint32_t rtcBootCount    = 0;
static RTC_DATA_ATTR bool     rtcModemAlive   = false;  // modem encendido y sin +CPOF
static RTC_DATA_ATTR bool     rtcInCutoff     = false;
static RTC_DATA_ATTR uint32_t rtcSleptSeconds = 0;
static RTC_DATA_ATTR uint32_t rtcAwakeMs      = 0;      // fraccion de segundo acumulada
static RTC_DATA_ATTR uint8_t  rtcStrikes      = 0;

static inline float cutoffV()  { return BAT_CUTOFF_V  + TEST_BAT_OFFSET_V; }
static inline float recoverV() { return BAT_RECOVER_V + TEST_BAT_OFFSET_V; }
static inline float warnV()    { return BAT_WARN_V    + TEST_BAT_OFFSET_V; }

static bool pmVbusPresent() { return readPinVolts() >= PIN_ON_V; }

// Unica puerta al deep sleep de todo el firmware.
//
// DEFECTO CORREGIDO (2026-08-20): antes ext0 solo se armaba si el pin estaba
// bajo en ese instante, y varios llamadores pasaban timerSeconds = 0. Con el
// pin alto por cualquier motivo (transitorio del buck, zona media, el cap de
// 100 nF todavia cargado, USB recien conectado) el resultado era un deep sleep
// SIN NINGUNA fuente de despertar: el equipo quedaba muerto hasta que se le
// quitara la alimentacion a mano. Ahora es imposible salir de aqui sin al
// menos una fuente armada, y se deja constancia en el serial.
static void pmDeepSleep(bool wakeOnIgnition, uint32_t timerSeconds) {
  bool ext0Armed = false;

  // ext0 solo si el pin esta BAJO ahora: si esta alto, despertaria de inmediato.
  if (wakeOnIgnition && !pmVbusPresent()) {
    esp_sleep_enable_ext0_wakeup((gpio_num_t)SENSE_PIN, 1);
    ext0Armed = true;
  }

  if (timerSeconds == 0 && !ext0Armed) {
    timerSeconds = SLEEP_FALLBACK_S;
    SerialMon.println("[PM] sin ext0 -> temporizador de respaldo para no quedar dormido para siempre");
  }

  if (timerSeconds > 0) {
    esp_sleep_enable_timer_wakeup((uint64_t)timerSeconds * 1000000ULL);
  }

  SerialMon.printf("[PM] deep sleep ext0=%d timer=%lus\n",
                   (int)ext0Armed, (unsigned long)timerSeconds);
  SerialMon.flush();
  esp_deep_sleep_start();  // no retorna
}

static void pmDisconnectClean() {
  if (mqtt.connected()) mqtt.disconnect();  // cierre limpio: no dispara el LWT
  netClient.stop();                         // y mata el socket zombi state=-4
}

// ---------------- Modem: dormir y despertar por DTR ----------------
static bool pmModemSleep() {
  modem.sendAT("+CSCLK=1");
  if (modem.waitResponse(1000) != 1) {
    SerialMon.println("[PM] AT+CSCLK=1 rechazado -> el modem queda despierto");
    rtcModemAlive = true;
    return false;                 // degrada, no rompe
  }
  pinMode(MODEM_DTR_PIN, OUTPUT);
  digitalWrite(MODEM_DTR_PIN, HIGH);          // DTR alto -> el modem duerme
  gpio_hold_en((gpio_num_t)MODEM_DTR_PIN);    // CLAVE: sostener en deep sleep
  gpio_deep_sleep_hold_en();                  // si no, el pin flota y despierta
  rtcModemAlive = true;
  return true;
}

static void pmModemWake() {
  // El hold se libera UNICAMENTE aqui. Soltarlo al inicio de setup() dejaria el
  // DTR flotando en los repasos de parqueo y el modem despertaria solo.
  gpio_deep_sleep_hold_dis();
  gpio_hold_dis((gpio_num_t)MODEM_DTR_PIN);
  pinMode(MODEM_DTR_PIN, OUTPUT);
  digitalWrite(MODEM_DTR_PIN, LOW);
  delay(50);
  for (int i = 0; i < 15 && !modem.testAT(500); i++) {
    watchdogFeed();
    delay(200);
  }
  modem.sendAT("+CSCLK=0");
  modem.waitResponse(1000);
}

// DEFECTO CORREGIDO (2026-08-20): pmParkedTick() llamaba pmModemWake() aun
// cuando el modem estaba apagado de verdad (rtcModemAlive = false tras un
// +CPOF). Bajar el DTR no revive un modem apagado: hay que pulsar PWRKEY. Y al
// contrario, pulsar PWRKEY con el modem encendido lo APAGA. Esta funcion es la
// unica que decide entre las dos cosas.
static void pmModemResume() {
  if (rtcModemAlive) {
    SerialMon.println("[PM] modem vivo -> despertar por DTR");
    pmModemWake();
  } else {
    SerialMon.println("[PM] modem apagado -> PWRKEY");
    modemPowerOn();
    waitForAT();
    rtcModemAlive = true;
  }
}

// AT crudo a proposito: la firma de disableGPS() en el fork de lewisxhe no esta
// verificada, y AT+CGNSSPWR=0 si esta comprobado en banco.
static void pmGnssOff() {
  modem.sendAT("+CGNSSPWR=0");
  modem.waitResponse(10000);
}

// TOQUE 4 (2026-08-22). El cuerpo vive ahora en include/gnss_prod.h.
//
// Lo que habia aqui era un bucle de 20 intentos de modem.enableGPS(), que
// espera hasta 30 s el URC "+CGNSSPWR: READY!". Ese URC NO EXISTE en el
// firmware SIM767XM5_B05V01_241206 (comprobado en el laboratorio GNSS v1.1,
// v1.2 y v1.4), asi que enableGPS() devolvia false SIEMPRE y el bucle daba sus
// veinte vueltas completas: 20 x (30 s + 0,5 s) = MAS DE DIEZ MINUTOS de
// setup() bloqueado en cada arranque.
//
// Durante esos diez minutos loop() no corria: no se muestreaba la ignicion, no
// se reintentaba MQTT, no se publicaba una sola posicion, y el engine_on salia
// con diez minutos de retraso. El watchdog de 120 s no rescataba nada porque el
// bucle alimentaba watchdogFeed() en cada vuelta. Y al final imprimia "no
// encendio tras 20 intentos" sobre un GNSS que llevaba diez minutos encendido y
// contestando AT+CGNSSINFO sin problema. Ver la SEXTA REGLA en gnss_prod.h.
static bool pmGnssOn() {
  return gnssPwrOn();
}

// ---------------- Guardian de arranque ----------------
// Se llama DESPUES de adcSetup(): sin la atenuacion configurada Y sin el
// calentamiento del ADC, la lectura de bateria sale mal y el guardian decidiria
// sobre un numero inventado. Eso no es teorico: paso en banco el 2026-08-20 y
// era la causa raiz del bug de esta rama. Ver el comentario de adcSetup().
//
// DEFECTO CORREGIDO (2026-08-20): el guardian usaba el umbral de REARRANQUE
// (3.80 V) como piso de arranque, sin histeresis. Una celda perfectamente sana
// en 3.7 V (~40 % de carga) con el carro parqueado hacia que el equipo se
// durmiera en el boot sin encender el modem y sin temporizador: tracker mudo
// justo en la mitad de la curva de descarga, y encima invisible. La histeresis
// correcta necesita las DOS puntas: se bloquea bajo el corte (3.50 V), y solo
// se exige el rearranque (3.80 V) cuando venimos de un corte real, que es lo
// que rtcInCutoff recuerda a traves del deep sleep. Y se duerme con repaso
// horario, para poder notar la recuperacion en vez de esperar la ignicion.
static void pmBootGuard() {
  rtcBootCount++;

  float v    = readBatteryVolts();
  bool  vbus = pmVbusPresent();

  SerialMon.printf("[PM] boot#%lu wake=%d bat=%.2f vbus=%d corte=%.2f rearranque=%.2f corte_previo=%d\n",
                   (unsigned long)rtcBootCount, (int)wakeCause,
                   v, (int)vbus, cutoffV(), recoverV(), (int)rtcInCutoff);

  // SEGUNDA OPINION antes de tomar una decision irreversible.
  //
  // Dormir una hora sin encender el modem es lo mas grave que hace este
  // firmware, y no se toma con una sola muestra. Si la primera lectura es
  // implausible o dice que no hay VBUS, se vuelve a medir. Cuesta 100 ms y solo
  // cuando algo se ve raro. El calentamiento de adcSetup() deberia hacer esto
  // innecesario, pero es exactamente el tipo de fallo que ya nos costo una
  // sesion completa: aqui se paga barato y se duerme tranquilo.
  if (!vbus || !batteryReadingPlausible(v)) {
    delay(100);
    v    = readBatteryVolts();
    vbus = pmVbusPresent();
    SerialMon.printf("[PM] segunda lectura bat=%.2f vbus=%d\n", v, (int)vbus);
  }

  if (!batteryReadingPlausible(v)) {
    SerialMon.printf("[PM] lectura de bateria implausible (%.2fV) -> no se bloquea el arranque\n", v);
    rtcInCutoff = false;
    return;
  }

  bool belowFloor    = (v <= cutoffV());
  bool stillInCutoff = (rtcInCutoff && v < recoverV());

  if (!vbus && (belowFloor || stillInCutoff)) {
#if TEST_DISABLE_SLEEP
    SerialMon.println("[PM] celda bajo el piso, pero TEST_DISABLE_SLEEP=1 -> se sigue de todas formas");
#else
    SerialMon.println("[PM] celda bajo el piso -> deep sleep SIN encender modem");
    rtcInCutoff   = true;
    rtcModemAlive = false;
    pmDeepSleep(true, GUARD_RECHECK_S);   // ext0 + repaso horario
#endif
  }
  rtcInCutoff = false;
}

// ---------------- Corte por bajo voltaje ----------------
static bool pmCheckCutoff() {
  static uint32_t last = 0;
  static bool warnSent = false;
  if (millis() - last < BAT_SAMPLE_MS) return false;
  last = millis();

  if (pmVbusPresent()) { rtcStrikes = 0; return false; }  // cargando: no cortar

  float v = readBatteryVolts();
  if (!batteryReadingPlausible(v)) {
    SerialMon.printf("[PM] bat=%.2f implausible -> no cuenta para el corte\n", v);
    rtcStrikes = 0;
    return false;
  }

  if (v <= cutoffV()) rtcStrikes++; else rtcStrikes = 0;

  // Aviso temprano, una sola vez al cruzar BAT_WARN_V; se rearma al recuperar.
  if (v <= warnV() && !warnSent) {
    publishStatus("low_battery");
    warnSent = true;
  } else if (v > warnV()) {
    warnSent = false;
  }

  SerialMon.printf("[PM] bat=%.2f strikes=%u/%u\n", v, rtcStrikes, BAT_CUTOFF_N);
  return (rtcStrikes >= BAT_CUTOFF_N);
}

static void pmEnterCutoff(float v) {
  char p[64];
  snprintf(p, sizeof(p), "low_battery_shutdown,%.2f", v);
  publishStatus(p);

  snprintf(p, sizeof(p), "%.2f", v);
  mqtt.publish(TOPIC_BATTERY, p, true);

  // event != "-" hace que el subscriber salte rate-limit y filtro de movimiento
  publishPoint(lastValidPoint, "low_battery");

  delay(SETTLE_MS);
  mqtt.loop();

  pmDisconnectClean();
  pmGnssOff();
  modem.sendAT("+CPOF");          // en corte se apaga TODO: manda la celda
  modem.waitResponse(10000);

  rtcModemAlive = false;
  rtcInCutoff   = true;
  rtcStrikes    = 0;

  SerialMon.println("[PM] corte por bajo voltaje -> deep sleep hasta ver VBUS");
  pmDeepSleep(true, GUARD_RECHECK_S);
}

// ---------------- Nivel 2: parqueo ----------------
// El engine_off ya lo publico serviceEvents(); aqui NO se repite.
static void pmEnterParked() {
  // Este log va ANTES de cualquier publicacion: si MQTT esta caido, el parqueo
  // no deja ninguna huella en el broker, y sin esta linea era indistinguible
  // de un cuelgue (2026-08-20).
  SerialMon.printf("[PM] entrando a parqueo mqtt=%d bat=%.2f\n",
                   (int)mqtt.connected(), readBatteryVolts());

  char b[16];
  snprintf(b, sizeof(b), "%.2f", readBatteryVolts());
  mqtt.publish(TOPIC_BATTERY, b, true);
  publishStatus("parked_sleep");

  delay(SETTLE_MS);
  mqtt.loop();

  pmDisconnectClean();
  pmGnssOff();                    // apaga GNSS, NO el modem
  pmModemSleep();                 // DTR: conserva registro LTE y efemerides
  rtcSleptSeconds = 0;
  rtcAwakeMs      = 0;

  SerialMon.println("[PM] parqueado -> deep sleep");
  pmDeepSleep(true, PARKED_POLL_S);
}

static void pmParkedTick() {
  // Estamos aqui por un despertar de temporizador con el pin de ignicion bajo:
  // por definicion, el carro esta apagado. Afirmarlo es cinturon y tirantes
  // sobre el ignState que ya viaja en RTC, y cubre el caso en que la RTC memory
  // se perdiera (reset de hardware con la placa parqueada, brownout).
  //
  // No es cosmetico: de este estado depende el campo ignition del CSV, y con
  // ignition=1 el subscriber tira el pulso con "Skip: no_move". Los 5 pulsos
  // de la noche del 19 al 20 se perdieron exactamente asi.
  ignState = IGN_OFF;

  // El reloj de parqueo cuenta sueno NOMINAL mas tiempo REALMENTE despierto.
  //
  // DEFECTO CORREGIDO (2026-08-20): antes solo sumaba PARKED_POLL_S, ignorando
  // lo que cuesta cada despertar. Medido esa noche: pulsos pedidos a 60 min
  // salieron a 67 min con desviacion de un segundo entre ciclos, +11.8 %,
  // porque cada ciclo nominal de 30 s duraba 33.5 s reales. En produccion eso
  // convertia las 24 h del pulso en casi 27. La fraccion de segundo se acumula
  // en rtcAwakeMs para que no se pierda redondeando en cada vuelta.
  rtcAwakeMs += millis();
  rtcSleptSeconds += PARKED_POLL_S + (rtcAwakeMs / 1000);
  rtcAwakeMs %= 1000;

  float v = readBatteryVolts();

  uint32_t pulse = (TEST_PULSE_S > 0) ? TEST_PULSE_S : PARKED_PULSE_S;
  SerialMon.printf("[PM] repaso parqueo dormido=%lus/%lus bat=%.2f\n",
                   (unsigned long)rtcSleptSeconds, (unsigned long)pulse, v);

  // El corte tambien vigila dormido, y aqui la lectura es limpia: no hay
  // rafagas LTE hundiendo el riel, asi que no hay falsos positivos por sag.
  if (!pmVbusPresent() && batteryReadingPlausible(v)) {
    if (v <= cutoffV()) rtcStrikes++; else rtcStrikes = 0;
    if (rtcStrikes >= BAT_CUTOFF_N) {
      pmModemResume();
      ensureLTE();
      if (tryConnectMQTT()) pmEnterCutoff(v);   // no retorna
      pmGnssOff();
      modem.sendAT("+CPOF");
      modem.waitResponse(10000);
      rtcModemAlive = false;
      rtcInCutoff   = true;
      pmDeepSleep(true, GUARD_RECHECK_S);
    }
  }

  if (rtcSleptSeconds >= pulse) {
    SerialMon.printf("[PM] pulso de bateria bat=%.2f\n", v);
    pmModemResume();
    ensureLTE();
    if (tryConnectMQTT()) {
      char b[16];
      snprintf(b, sizeof(b), "%.2f", v);
      mqtt.publish(TOPIC_BATTERY, b, true);
      publishStatus("parked_pulse");
      // Si un parqueo forzado (MQTT caido) se llevo el engine_off a dormir,
      // este es el momento de sacarlo: llega tarde, pero llega.
      serviceEvents();
      // El "sigo aqui" para Traccar: mismo contrato que el keepalive del
      // Nivel 1. ignition=0 en el CSV hace que el subscriber lo deje pasar,
      // y ese 0 sale de ignState, que ahora vive en RTC y se afirma arriba.
      publishPoint(lastValidPoint, EVENT_NONE);
      delay(SETTLE_MS);
      mqtt.loop();
      pmDisconnectClean();
    } else {
      SerialMon.println("[PM] pulso sin MQTT -> se reintenta en el siguiente");
    }
    pmModemSleep();
    rtcSleptSeconds = 0;
    rtcAwakeMs      = 0;
  }

  pmDeepSleep(true, PARKED_POLL_S);
}
