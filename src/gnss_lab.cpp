// =============================================================================
//  GPS Tracker Logan  ·  GNSS LAB v1.2  ·  src/gnss_lab.cpp  ·  env: gnss_lab
// =============================================================================
//  QUE CAMBIO RESPECTO A LA v1.1 (corrida del 2026-08-22, log en logs/)
//
//  1. SE MURIO LA ESPERA DEL READY FANTASMA.
//     La v1.1 esperaba 30 s el URC "+CGNSSPWR: READY!" despues de CGNSSPWR=1.
//     En las 7 corridas del log ese URC NO llego nunca y el modem no mando un
//     solo byte en esos 30 s: este firmware (SIM767XM5_B05V01_241206) no lo
//     emite. Consecuencia: el primer sondeo caia en t=30 s y tres corridas
//     marcaron exactamente 30 s, o sea "30 s o menos", sin saber cuanto menos.
//     Ahora se espera 2 s por cortesia y se empieza a sondear de una, cada 1 s.
//     Esto ademas ensaya en el banco el cambio principal de F19 antes de
//     meterlo en main.cpp: TinyGSM enableGPS() se cuelga en esa misma espera y
//     devuelve false, regalando 30 s en cada encendido de produccion.
//
//  2. DESEMPATE DEL SUENO POR DTR.
//     En la v1.1, R3 (sueno por DTR) tardo 120 s, casi igual que el arranque en
//     frio real (117 s), mientras el control tardo 30 s o menos. Pero R3 tuvo
//     el GNSS apagado 277 s contra 150 s de las demas corridas: el sueno se
//     sumaba al enfriado. Habia dos explicaciones y un solo dato.
//       T1 warm_ctrl : GNSS apagado 150 s, nada mas.        Referencia.
//       T2 long_off  : GNSS apagado 277 s, modem despierto. Aisla el tiempo.
//       T3 dtr_sleep : GNSS apagado 277 s, 120 s dormido.   Repite R3.
//     Si T2 sale corto y T3 largo, el culpable es el sueno.
//     Si T2 y T3 salen largos, el culpable es el tiempo y el sueno es inocente.
//     Si los tres salen cortos, R3 fue ruido de una sola muestra en interiores.
//
//  3. SE PRUEBAN CGPSHOT Y CGPSWARM.
//     Hallazgo de la investigacion: el manual A76XX (y el SIM7500_SIM7600
//     seccion 17.3-17.4) documenta AT+CGPSCOLD, AT+CGPSWARM y AT+CGPSHOT, que
//     arrancan la sesion GNSS forzando el modo de arranque. No son los
//     CGNSSxxx que veniamos usando, por eso AT+CGNSSCOLD dio ERROR en la v1.1.
//     En el issue 453 de LilyGo-Modem-Series, con este mismo modulo y este
//     mismo firmware, un usuario reporta pasar de 5-10 min a 15-60 s anadiendo
//     CGNSSMODE=15 y CGPSWARM. Regla del manual: deben mandarse con el motor
//     GNSS apagado, que es justo el estado en el que arranca cada corrida.
//       T4 hot_cmd  : AT+CGPSHOT  en vez de CGNSSPWR=1.
//       T5 warm_cmd : AT+CGPSWARM en vez de CGNSSPWR=1.
//     Si alguno existe y baja el TTFF, vale mas que todo lo demas junto.
//     Si el comando no existe o no arranca el motor, la corrida cae sola a
//     CGNSSPWR=1, lo anota y sigue midiendo (no se pierde la corrida).
//
//  LO QUE YA QUEDO RESUELTO Y POR ESO NO SE VUELVE A MEDIR
//    - CRESET no borra nada caro (42 s) y el modem contesta en 1 s.
//    - El pulso de PWRKEY sobre modem vivo NO lo tumba.
//    - CGNSSMODE es SAVE-persistente: sobrevive sin reescribirlo.
//    - CGNSSMODE? con el GNSS apagado siempre da ERROR: el manual dice que el
//      comando solo es valido con el GNSS encendido. Ya no se pregunta antes.
//    - Escribir CGNSSMODE=15 cuesta ~10 s y suelta una trama envenenada con
//      lat/lon viejas y sats vacios. Por eso el gate sats >= 5 es obligatorio.
//    Esas corridas quedan definidas abajo pero desactivadas, con su resultado
//    anotado, por si algun dia hay que repetirlas.
//
//  DATO DE CONTEXTO PARA LEER EL RESULTADO (issue 453, respuesta de LilyGO)
//    "Warm start relies on an external RTC backup power supply, so the modem
//     must be powered on for a warm start to work. If the modem is turned off,
//     the ephemeris data will be lost."
//    O sea: las efemerides viven en la RAM del modulo alimentada, NO en flash.
//    Apagar el modem (CPOF o corte de energia) las pierde siempre. Dormirlo
//    por DTR en teoria NO deberia perderlas, porque el modulo sigue
//    alimentado. Justo eso es lo que T2 y T3 van a resolver con datos.
//
//  CONDICIONES IGUALADAS CON main.cpp
//    - UART1 a 115200 en los mismos pines (RX 5 / TX 4).
//    - Encendido del modem identico a modemPowerOn(): DTR LOW, PWRKEY LOW,
//      100 ms, HIGH, 1000 ms, LOW.  OJO: at_passthrough.cpp tiene la polaridad
//      invertida; aqui se usa la de main.cpp, que es la de LilyGo.
//    - Gate de calidad de main.cpp: sats >= 5 y HDOP <= 2.5. Se cronometran
//      por separado el primer fix crudo y el primero que pasa el gate.
//    - Lectura de bateria con warmup de 8 muestras descartadas, como
//      ADC_WARMUP_READS en main.cpp: en la v1.1 la primera lectura dio 3.57 V
//      y las cinco siguientes 4.18 V. Sin warmup el ADC miente en la primera.
//
//  ANTES DE FLASHEAR
//    1. Desconecta USB y saca la 18650 unos 10 s.
//    2. pio run -e gnss_lab -t upload
//       pio device monitor -e gnss_lab
//    3. Antena GNSS quieta, en el mismo sitio de la corrida anterior si se
//       quiere comparar contra el log del 2026-08-22.
//    Duracion tipica 25-30 min; techo ~45 min.
//
//  AL TERMINAR, VOLVER A PRODUCCION:  pio run -e tracker -t upload
// =============================================================================

#include <Arduino.h>
#include <esp_system.h>
#include <stdarg.h>

// ------------------------------------------------------- pines (iguales a main)
#define MODEM_TX_PIN                  4      // ESP32 -> modem
#define MODEM_RX_PIN                  5      // modem -> ESP32
#define MODEM_DTR_PIN                 7      // RTC-capable, sirve para hold
#define BOARD_PWRKEY_PIN              46     // strapping, NO RTC-capable
#define MODEM_POWERON_PULSE_WIDTH_MS  1000
#define BAT_ADC_PIN                   8
#define BAT_DIVIDER_FACTOR            2.0f
#define ADC_WARMUP_READS              8

// ------------------------------------------------- gate de calidad de main.cpp
#define MIN_VALID_SATELLITES          5
#define MAX_VALID_HDOP                2.5f

// ------------------------------------------------------- parametros del banco
#define LAB_VERSION                   "gnss_lab 1.2"
#define FIX_TIMEOUT_S                 300UL   // techo por corrida
#define PREFLIGHT_TIMEOUT_S           240UL   // techo del baseline de cielo
#define POSTFIX_GRACE_S               60UL    // margen para pasar el gate
#define MAX_RECHARGES                 2       // baselines de rescate permitidos
#define POLL_PERIOD_MS                1000UL  // cadencia de CGNSSINFO
#define POLL_LOG_EVERY                5       // imprime 1 de cada N sondeos
#define READY_WAIT_MS                 2000UL  // cortesia: este firmware no manda READY
#define GNSS_OFF_WAIT_S               150UL   // GNSS apagado entre corridas
#define LONG_OFF_EXTRA_S              127UL   // extra de T2: iguala los 277 s de T3
#define DTR_SLEEP_S                   120UL   // duracion del sueno por DTR
#define DTR_SLEEP_SHORT_S             20UL    // sueno corto (corrida opcional)
#define START_COUNTDOWN_S             10UL
#define SERIAL_WAIT_MS                8000UL

HardwareSerial SerialAT(1);

// =============================================================================
//  Definicion de las corridas
// =============================================================================
enum Pre : uint8_t {
  PRE_NONE,             // nada: control
  PRE_LONG_OFF,         // espera extra con el GNSS apagado y el modem despierto
  PRE_DTR_SLEEP,        // CSCLK=1 + DTR alto + despertar
  PRE_DTR_SLEEP_SHORT,  // igual pero corto: separa el evento de la duracion
  PRE_CSCLK_AWAKE,      // CSCLK=1 con DTR bajo: configura pero no duerme
  PRE_CPOF_PWRKEY,      // apagado real del modem: arranque frio de referencia
  PRE_CRESET,           // AT+CRESET
  PRE_PWRKEY_PULSE      // pulso de PWRKEY sobre modem vivo
};

enum Start : uint8_t {
  START_PWR,   // CGDRT + CGSETV + CGNSSPWR=1   (lo que hace pmGnssOn)
  START_HOT,   // CGDRT + CGSETV + CGPSHOT
  START_WARM   // CGDRT + CGSETV + CGPSWARM
};

enum Post : uint8_t {
  POST_NONE,          // nada mas
  POST_BAUD,          // + CGNSSIPR=115200
  POST_BAUD_MODE      // + CGNSSIPR=115200 + CGNSSMODE=15
};

struct RunCfg {
  const char *id;
  const char *label;
  bool        enabled;
  Pre         pre;
  Start       start;
  Post        post;
};

static const RunCfg RUNS[] = {
  // --- desempate del sueno por DTR ------------------------------------------
  { "T1", "warm_ctrl",   true,  PRE_NONE,            START_PWR,  POST_NONE },
  { "T2", "long_off",    true,  PRE_LONG_OFF,        START_PWR,  POST_NONE },
  { "T3", "dtr_sleep",   true,  PRE_DTR_SLEEP,       START_PWR,  POST_NONE },
  // --- comandos de arranque que aparecieron en la investigacion -------------
  { "T4", "hot_cmd",     true,  PRE_NONE,            START_HOT,  POST_NONE },
  { "T5", "warm_cmd",    true,  PRE_NONE,            START_WARM, POST_NONE },
  // --- segunda vuelta, activar solo si hace falta ---------------------------
  // T6: si T3 sale largo, dice si lo rompe el evento de dormir o la duracion.
  { "T6", "sleep_short", false, PRE_DTR_SLEEP_SHORT, START_PWR,  POST_NONE },
  // T7: si T3 sale largo, separa configurar CSCLK de dormir de verdad.
  { "T7", "csclk_awake", false, PRE_CSCLK_AWAKE,     START_PWR,  POST_NONE },
  // T8: si T3 sale largo Y T4/T5 funcionan, esta es la receta de produccion.
  { "T8", "hot_tras_sueno", false, PRE_DTR_SLEEP,    START_HOT,  POST_NONE },
  // --- ya respondidas en la corrida del 2026-08-22 --------------------------
  { "R0", "cold_ref",    false, PRE_CPOF_PWRKEY,     START_PWR,  POST_BAUD_MODE }, // 117 s
  { "R2", "creset",      false, PRE_CRESET,          START_PWR,  POST_NONE },      // 42 s
  { "R4", "cgnssmode15", false, PRE_NONE,            START_PWR,  POST_BAUD_MODE }, // 40 s + trama fantasma
  { "R5", "pwrkey_live", false, PRE_PWRKEY_PULSE,    START_PWR,  POST_NONE },      // no tumba el modem
};
static const int RUN_COUNT = sizeof(RUNS) / sizeof(RUNS[0]);

struct RunResult {
  bool     ran        = false;
  bool     fixRaw     = false;
  bool     fixValid   = false;
  bool     ready      = false;
  uint32_t ttffRawMs  = 0;
  uint32_t ttffValMs  = 0;
  uint32_t offTotalS  = 0;
  int      sats       = 0;
  float    hdop       = -1.0f;
  uint16_t pollsEmpty = 0;
  uint16_t pollsError = 0;
  uint16_t pollsTotal = 0;
  float    batV       = 0.0f;
  String   modeAfter  = "-";
  String   note       = "-";
};

static RunResult results[RUN_COUNT];
static bool      labDone = false;

// estado del cielo medido por el baseline
static bool     skyOk         = false;
static uint32_t skyTtffMs     = 0;
static int      skySats       = 0;
static float    skyHdop       = -1.0f;
static int      rechargesUsed = 0;
static String   abortReason   = "";
static String   cmdSupport    = "";   // que contesto cada comando de arranque
static uint32_t gnssOffSinceMs = 0;   // cuando se apago el GNSS por ultima vez

// =============================================================================
//  Utilidades de log
// =============================================================================
static void logf(const char *fmt, ...) {
  char buf[512];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  uint32_t ms = millis();
  Serial.printf("[%5lu.%lus] %s\r\n",
                (unsigned long)(ms / 1000UL),
                (unsigned long)((ms % 1000UL) / 100UL),
                buf);
}

static String oneLine(const String &in) {
  String o = in;
  o.replace("\r\n", " | ");
  o.replace("\r", " ");
  o.replace("\n", " | ");
  while (o.indexOf("|  |") >= 0) o.replace("|  |", "|");
  o.trim();
  if (o.length() == 0) o = "(sin respuesta)";
  return o;
}

static const char *preName(Pre p) {
  switch (p) {
    case PRE_LONG_OFF:        return "long_off";
    case PRE_DTR_SLEEP:       return "dtr_sleep";
    case PRE_DTR_SLEEP_SHORT: return "dtr_sleep_20s";
    case PRE_CSCLK_AWAKE:     return "csclk_awake";
    case PRE_CPOF_PWRKEY:     return "cpof+pwrkey";
    case PRE_CRESET:          return "creset";
    case PRE_PWRKEY_PULSE:    return "pwrkey_pulse";
    default:                  return "nada";
  }
}

static const char *startName(Start s) {
  switch (s) {
    case START_HOT:  return "cgpshot";
    case START_WARM: return "cgpswarm";
    default:         return "cgnsspwr";
  }
}

static const char *startCmd(Start s) {
  switch (s) {
    case START_HOT:  return "AT+CGPSHOT";
    case START_WARM: return "AT+CGPSWARM";
    default:         return "AT+CGNSSPWR=1";
  }
}

static const char *resetReasonName() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_EXT:       return "EXT";
    case ESP_RST_SW:        return "SW";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "UNKNOWN";
  }
}

// Warmup igual que main.cpp: la primera lectura del ADC no sirve.
static float readBatV() {
  for (int i = 0; i < ADC_WARMUP_READS; i++) {
    (void)analogReadMilliVolts(BAT_ADC_PIN);
    delay(2);
  }
  uint32_t acc = 0;
  for (int i = 0; i < 16; i++) {
    acc += analogReadMilliVolts(BAT_ADC_PIN);
    delay(2);
  }
  return (acc / 16.0f) * BAT_DIVIDER_FACTOR / 1000.0f;
}

// =============================================================================
//  Capa AT
// =============================================================================
static String atSend(const char *cmd, uint32_t timeoutMs, bool quiet) {
  while (SerialAT.available()) SerialAT.read();   // drena basura previa
  if (!quiet) logf("[AT  ] > %s", cmd);
  SerialAT.print(cmd);
  SerialAT.print("\r\n");

  String   buf;
  buf.reserve(256);
  uint32_t t0   = millis();
  bool     done = false;
  while (millis() - t0 < timeoutMs) {
    while (SerialAT.available()) {
      buf += (char)SerialAT.read();
      if (buf.length() > 1536) buf.remove(0, 512);
    }
    if (buf.indexOf("OK") >= 0 || buf.indexOf("ERROR") >= 0) { done = true; break; }
    delay(5);
  }
  if (!quiet) {
    logf("[AT  ] < %s   (%lu ms%s)", oneLine(buf).c_str(),
         (unsigned long)(millis() - t0), done ? "" : ", TIMEOUT");
  }
  return buf;
}

static bool atOk(const char *cmd, uint32_t timeoutMs) {
  return atSend(cmd, timeoutMs, false).indexOf("OK") >= 0;
}

static bool modemAnswers(uint32_t timeoutMs) {
  return atSend("AT", timeoutMs, true).indexOf("OK") >= 0;
}

static bool waitModemAt(uint32_t timeoutMs) {
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    if (modemAnswers(1000)) {
      logf("[MDM ] responde AT tras %lu ms", (unsigned long)(millis() - t0));
      return true;
    }
    delay(500);
  }
  logf("[MDM ] SIN respuesta AT tras %lu ms", (unsigned long)(millis() - t0));
  return false;
}

static String fieldAfter(const String &resp, const char *tag) {
  int p = resp.indexOf(tag);
  if (p < 0) return String(resp.indexOf("ERROR") >= 0 ? "ERROR" : "?");
  String s = resp.substring(p + strlen(tag));
  int e = s.indexOf('\r');
  if (e < 0) e = s.indexOf('\n');
  if (e >= 0) s = s.substring(0, e);
  s.trim();
  if (s.length() == 0) s = "(vacio)";
  return s;
}

// =============================================================================
//  Modem: encendido, apagado, reset, sueno por DTR
// =============================================================================
static void pwrkeyPulse() {
  logf("[MDM ] pulso PWRKEY (polaridad main.cpp: LOW-HIGH %d ms-LOW)",
       MODEM_POWERON_PULSE_WIDTH_MS);
  pinMode(BOARD_PWRKEY_PIN, OUTPUT);
  digitalWrite(BOARD_PWRKEY_PIN, LOW);
  delay(100);
  digitalWrite(BOARD_PWRKEY_PIN, HIGH);
  delay(MODEM_POWERON_PULSE_WIDTH_MS);
  digitalWrite(BOARD_PWRKEY_PIN, LOW);
}

static void echoOff() { atSend("ATE0", 2000, false); }

static String modemColdCycle() {
  String note = "-";
  logf("[MDM ] --- apagado real del modem (AT+CPOF) ---");
  if (!modemAnswers(1500)) {
    note = "ya estaba mudo antes del CPOF";
    logf("[MDM ] %s", note.c_str());
  } else {
    atSend("AT+CPOF", 8000, false);
    uint32_t t0 = millis();
    bool off = false;
    while (millis() - t0 < 25000UL) {
      if (!modemAnswers(800)) { off = true; break; }
      delay(500);
    }
    logf("[MDM ] modem %s tras %lu ms", off ? "APAGADO" : "sigue contestando",
         (unsigned long)(millis() - t0));
    if (!off) note = "CPOF no apago el modem";
    delay(3000);
  }
  pwrkeyPulse();
  if (!waitModemAt(60000UL)) note = "no revivio tras el pulso";
  delay(2000);
  echoOff();
  return note;
}

static String modemReset() {
  logf("[MDM ] --- AT+CRESET ---");
  atSend("AT+CRESET", 6000, false);
  delay(1000);
  uint32_t t0 = millis();
  bool back = waitModemAt(60000UL);
  String note = back
    ? String("AT vuelve ") + String((unsigned long)((millis() - t0) / 1000)) + "s tras CRESET"
    : String("el modem no volvio tras CRESET");
  delay(2000);
  echoOff();
  return note;
}

// Espera con el GNSS ya apagado y el modem despierto. Es el gemelo del sueno:
// misma duracion, sin dormir. Si T2 sale corto, el tiempo es inocente.
static String longOffWait() {
  logf("[MDM ] --- espera extra de %lu s con el modem DESPIERTO ---",
       (unsigned long)LONG_OFF_EXTRA_S);
  uint32_t t0 = millis();
  while (millis() - t0 < LONG_OFF_EXTRA_S * 1000UL) {
    delay(1000);
    uint32_t el = (millis() - t0) / 1000UL;
    if (el % 30 == 0 && el > 0) {
      logf("[MDM ] esperando despierto %lu/%lu s",
           (unsigned long)el, (unsigned long)LONG_OFF_EXTRA_S);
    }
  }
  bool alive = modemAnswers(1500);
  return alive ? "espera larga sin dormir" : "espera larga y el modem quedo mudo";
}

// CSCLK=1 + DTR alto = el modem duerme. DTR bajo = despierta.
static String dtrSleepCycle(uint32_t seconds) {
  logf("[MDM ] --- sueno por DTR: CSCLK=1 + DTR HIGH %lu s ---",
       (unsigned long)seconds);
  atOk("AT+CSCLK=1", 3000);
  digitalWrite(MODEM_DTR_PIN, HIGH);
  delay(5000);

  bool awakeWithDtrHigh = modemAnswers(1500);
  logf("[MDM ] con DTR alto el modem %s",
       awakeWithDtrHigh ? "SIGUE contestando (no durmio)" : "no contesta (durmio)");

  uint32_t t0 = millis();
  while (millis() - t0 < seconds * 1000UL) {
    delay(1000);
    uint32_t el = (millis() - t0) / 1000UL;
    if (el % 30 == 0 && el > 0) logf("[MDM ] dormido por DTR %lu/%lu s",
                                     (unsigned long)el, (unsigned long)seconds);
  }

  digitalWrite(MODEM_DTR_PIN, LOW);
  delay(200);
  bool back = waitModemAt(20000UL);
  atOk("AT+CSCLK=0", 3000);
  if (!awakeWithDtrHigh && back) {
    return String("durmio ") + String((unsigned long)seconds) + "s y despertio por DTR";
  }
  if (awakeWithDtrHigh) return "no llego a dormir con DTR alto";
  return "no despertio por DTR";
}

// CSCLK=1 pero con DTR abajo: el modem queda configurado para dormir y no
// duerme. Si esto solo ya rompe el TTFF, el problema es el comando, no el sueno.
static String csclkAwake() {
  logf("[MDM ] --- CSCLK=1 con DTR BAJO durante %lu s (no debe dormir) ---",
       (unsigned long)LONG_OFF_EXTRA_S);
  atOk("AT+CSCLK=1", 3000);
  digitalWrite(MODEM_DTR_PIN, LOW);
  uint32_t t0 = millis();
  bool everMute = false;
  while (millis() - t0 < LONG_OFF_EXTRA_S * 1000UL) {
    delay(1000);
    uint32_t el = (millis() - t0) / 1000UL;
    if (el % 30 == 0 && el > 0) {
      bool alive = modemAnswers(1000);
      if (!alive) everMute = true;
      logf("[MDM ] %lu/%lu s con CSCLK=1 y DTR bajo: modem %s",
           (unsigned long)el, (unsigned long)LONG_OFF_EXTRA_S,
           alive ? "despierto" : "MUDO");
    }
  }
  atOk("AT+CSCLK=0", 3000);
  return everMute ? "CSCLK=1 lo durmio aun con DTR bajo" : "CSCLK=1 sin dormir";
}

static String pwrkeyOnLiveModem() {
  logf("[MDM ] --- pulso de PWRKEY sobre modem VIVO ---");
  pwrkeyPulse();
  delay(5000);
  bool alive = modemAnswers(2000);
  String note;
  if (alive) {
    note = "el pulso NO tumbo el modem";
    logf("[MDM ] %s", note.c_str());
  } else {
    note = "el pulso APAGO el modem";
    logf("[MDM ] %s -- lo reenciendo", note.c_str());
    delay(3000);
    pwrkeyPulse();
    if (!waitModemAt(60000UL)) note += "; y no revivio";
    delay(2000);
    echoOff();
  }
  return note;
}

// =============================================================================
//  GNSS
// =============================================================================
struct GnssSample {
  bool  hasTag = false;
  bool  hasFix = false;
  int   sats   = 0;
  float hdop   = -1.0f;
};

static GnssSample gnssPoll() {
  GnssSample s;
  String r = atSend("AT+CGNSSINFO", 3000, true);
  int p = r.indexOf("+CGNSSINFO:");
  if (p < 0) return s;                       // ERROR o timeout
  s.hasTag = true;

  String body = r.substring(p + 11);
  int e = body.indexOf('\r');
  if (e < 0) e = body.indexOf('\n');
  if (e >= 0) body = body.substring(0, e);
  body.trim();

  // +CGNSSINFO: mode,GPSsv,GLONASSsv,BEIDOUsv,GALILEOsv,lat,N/S,lon,E/W,
  //             date,utc,alt,speed,course,PDOP,HDOP,VDOP
  String f[17];
  int n = 0, start = 0;
  while (n < 17) {
    int c = body.indexOf(',', start);
    if (c < 0) { f[n++] = body.substring(start); break; }
    f[n++] = body.substring(start, c);
    start = c + 1;
  }
  s.sats = f[1].toInt() + f[2].toInt() + f[3].toInt() + f[4].toInt();
  s.hdop = (f[15].length() > 0) ? f[15].toFloat() : -1.0f;
  s.hasFix = (f[5].length() > 0 && f[7].length() > 0);
  return s;
}

static void gnssOff() {
  atSend("AT+CGNSSPWR=0", 5000, false);
  gnssOffSinceMs = millis();
}

// Arranca el motor GNSS y devuelve el instante exacto del comando: la
// referencia del TTFF. Ya NO se esperan 30 s por un URC que este firmware no
// manda; se dan 2 s de cortesia y se empieza a sondear.
static uint32_t gnssStart(Start st, bool &readyOut, String &noteOut) {
  atSend("AT+CGDRT=1,1", 3000, false);       // alimentacion de antena GNSS
  atSend("AT+CGSETV=1,1", 3000, false);

  const char *cmd = startCmd(st);
  uint32_t t0 = millis();
  String r = atSend(cmd, 8000, false);
  bool ok    = r.indexOf("OK") >= 0;
  bool ready = r.indexOf("READY") >= 0;

  if (!ready) {                              // ventana corta, solo para saber
    String acc;                              // si algun dia el URC aparece
    uint32_t w0 = millis();
    while (millis() - w0 < READY_WAIT_MS) {
      while (SerialAT.available()) acc += (char)SerialAT.read();
      if (acc.indexOf("READY") >= 0) { ready = true; break; }
      delay(10);
    }
    if (acc.length()) logf("[AT  ] < %s", oneLine(acc).c_str());
  }
  logf("[GNSS] %s -> %s%s tras %lu ms", cmd, ok ? "OK" : "SIN OK",
       ready ? " + READY" : " (sin READY, normal en este firmware)",
       (unsigned long)(millis() - t0));

  // Los comandos de arranque forzado pueden no existir o no encender el motor.
  // Se comprueba y, si hace falta, se cae al camino de siempre sin perder la
  // corrida: el cronometro se reinicia en el CGNSSPWR=1 real.
  if (st != START_PWR) {
    String pwr = fieldAfter(atSend("AT+CGNSSPWR?", 3000, false), "+CGNSSPWR:");
    bool engineOn = ok && pwr.indexOf("1") >= 0;
    logf("[GNSS] tras %s el motor esta %s (CGNSSPWR? = %s)",
         cmd, engineOn ? "ENCENDIDO" : "apagado", pwr.c_str());
    if (!engineOn) {
      noteOut = String(cmd) + (ok ? " no encendio el motor" : " no soportado");
      noteOut += "; medido con CGNSSPWR=1";
      logf("[GNSS] %s", noteOut.c_str());
      t0 = millis();
      atSend("AT+CGNSSPWR=1", 6000, false);
    } else {
      noteOut = String(cmd) + " arranco el motor";
    }
  }

  readyOut = ready;
  return t0;
}

static void coolDown(const char *tag) {
  logf("[%s  ] GNSS apagado, enfriando %lu s", tag, (unsigned long)GNSS_OFF_WAIT_S);
  uint32_t w0 = millis();
  while (millis() - w0 < GNSS_OFF_WAIT_S * 1000UL) {
    delay(1000);
    uint32_t el = (millis() - w0) / 1000UL;
    if (el % 30 == 0 && el > 0) {
      logf("[%s  ] enfriando %lu/%lu s", tag, (unsigned long)el,
           (unsigned long)GNSS_OFF_WAIT_S);
    }
  }
}

// =============================================================================
//  Baseline de cielo: exige un fix crudo antes de seguir.
// =============================================================================
static bool warmBaseline(const char *tag) {
  Serial.println();
  Serial.println("-----------------------------------------------------------------");
  logf("[%s  ] BASELINE DE CIELO: enciendo GNSS y espero fix crudo (max %lu s)",
       tag, (unsigned long)PREFLIGHT_TIMEOUT_S);
  Serial.println("-----------------------------------------------------------------");

  bool ready = false;
  String note = "-";
  uint32_t t0 = gnssStart(START_PWR, ready, note);

  uint32_t nextPoll = millis();
  int  idx = 0, bestSats = 0;
  float lastHdop = -1.0f;
  bool got = false;
  uint32_t ttff = 0;

  while (millis() - t0 < PREFLIGHT_TIMEOUT_S * 1000UL) {
    if (millis() < nextPoll) { delay(20); continue; }
    nextPoll = millis() + POLL_PERIOD_MS;
    idx++;
    GnssSample s = gnssPoll();
    if (s.sats > bestSats) bestSats = s.sats;
    if (s.hdop > 0) lastHdop = s.hdop;
    if (s.hasFix) { got = true; ttff = millis() - t0; break; }
    if (idx % POLL_LOG_EVERY == 0) {
      logf("[%s  ] t=%lus sats=%d hdop=%.1f (sin fix aun)", tag,
           (unsigned long)((millis() - t0) / 1000), s.sats, s.hdop);
    }
  }

  gnssOff();

  if (got) {
    skyOk     = true;
    skyTtffMs = ttff;
    skySats   = bestSats;
    skyHdop   = lastHdop;
    logf("[%s  ] fix crudo a los %lu s  sats=%d hdop=%.1f -> el cielo alcanza",
         tag, (unsigned long)(ttff / 1000), bestSats, lastHdop);
    coolDown(tag);
    return true;
  }

  logf("[%s  ] SIN fix crudo en %lu s. Mejor conteo de satelites: %d",
       tag, (unsigned long)PREFLIGHT_TIMEOUT_S, bestSats);
  return false;
}

// =============================================================================
//  Ejecucion de una corrida
// =============================================================================
static void runOne(const RunCfg &cfg, RunResult &out, bool coolAfter) {
  Serial.println();
  Serial.println("-----------------------------------------------------------------");
  logf("[%s  ] INICIO  %s   pre=%s  start=%s",
       cfg.id, cfg.label, preName(cfg.pre), startName(cfg.start));
  Serial.println("-----------------------------------------------------------------");

  out.ran  = true;
  out.batV = readBatV();
  logf("[%s  ] Vbat aprox %.2f V", cfg.id, out.batV);

  // ---- variable bajo prueba -------------------------------------------------
  switch (cfg.pre) {
    case PRE_LONG_OFF:        out.note = longOffWait();                    break;
    case PRE_DTR_SLEEP:       out.note = dtrSleepCycle(DTR_SLEEP_S);       break;
    case PRE_DTR_SLEEP_SHORT: out.note = dtrSleepCycle(DTR_SLEEP_SHORT_S); break;
    case PRE_CSCLK_AWAKE:     out.note = csclkAwake();                     break;
    case PRE_CPOF_PWRKEY:     out.note = modemColdCycle();                 break;
    case PRE_CRESET:          out.note = modemReset();                     break;
    case PRE_PWRKEY_PULSE:    out.note = pwrkeyOnLiveModem();              break;
    default:                  out.note = "-";                              break;
  }

  // Cuanto lleva el GNSS apagado justo antes de encenderlo: la variable que
  // confundia la lectura de R3 en la v1.1. Ahora queda medida y en el CSV.
  out.offTotalS = gnssOffSinceMs ? (millis() - gnssOffSinceMs) / 1000UL : 0;
  logf("[%s  ] GNSS lleva %lu s apagado al momento de encenderlo",
       cfg.id, (unsigned long)out.offTotalS);

  // ---- arranque del motor GNSS ---------------------------------------------
  bool ready = false;
  String startNote = "-";
  uint32_t t0 = gnssStart(cfg.start, ready, startNote);
  out.ready = ready;
  if (startNote != "-") {
    if (out.note == "-") out.note = startNote;
    else                 out.note += "; " + startNote;
  }

  if (cfg.post == POST_BAUD || cfg.post == POST_BAUD_MODE) {
    atSend("AT+CGNSSIPR=115200", 3000, false);
  }
  if (cfg.post == POST_BAUD_MODE) {
    atSend("AT+CGNSSMODE=15", 5000, false);
  }

  // ---- cronometro ----------------------------------------------------------
  uint32_t nextPoll      = millis();
  uint32_t graceDeadline = 0;
  int      pollIdx       = 0;
  int      lastSats      = -1;

  while (millis() - t0 < FIX_TIMEOUT_S * 1000UL) {
    if (millis() < nextPoll) { delay(20); continue; }
    nextPoll = millis() + POLL_PERIOD_MS;
    pollIdx++;
    out.pollsTotal++;

    GnssSample s = gnssPoll();
    if (!s.hasTag)      out.pollsError++;
    else if (!s.hasFix) out.pollsEmpty++;

    // Trama envenenada: lat/lon presentes pero sin satelites ni HDOP. Es la
    // ultima posicion conocida, no una medida. El gate de main.cpp la mata.
    bool poisoned = s.hasFix && (s.sats == 0 || s.hdop <= 0.0f);
    if (poisoned) {
      logf("[%s  ] t=%lus TRAMA FANTASMA: fix con sats=%d hdop=%.1f (posicion vieja)",
           cfg.id, (unsigned long)((millis() - t0) / 1000), s.sats, s.hdop);
    }

    bool passesGate = s.hasFix &&
                      s.sats >= MIN_VALID_SATELLITES &&
                      s.hdop > 0.0f && s.hdop <= MAX_VALID_HDOP;

    if (s.hasFix && !out.fixRaw) {
      out.fixRaw    = true;
      out.ttffRawMs = millis() - t0;
      graceDeadline = millis() + POSTFIX_GRACE_S * 1000UL;
      logf("[%s  ] *** FIX CRUDO a los %lu s   sats=%d hdop=%.1f",
           cfg.id, (unsigned long)(out.ttffRawMs / 1000), s.sats, s.hdop);
    }
    if (passesGate && !out.fixValid) {
      out.fixValid  = true;
      out.ttffValMs = millis() - t0;
      out.sats      = s.sats;
      out.hdop      = s.hdop;
      logf("[%s  ] *** FIX QUE PASA EL GATE a los %lu s   sats=%d hdop=%.1f",
           cfg.id, (unsigned long)(out.ttffValMs / 1000), s.sats, s.hdop);
      break;
    }

    bool interesting = (s.sats != lastSats) || (pollIdx % POLL_LOG_EVERY == 0);
    if (interesting) {
      logf("[%s  ] t=%lus poll=%d fix=%s sats=%d hdop=%.1f vacios=%u err=%u",
           cfg.id, (unsigned long)((millis() - t0) / 1000), pollIdx,
           s.hasFix ? "si" : "no", s.sats, s.hdop,
           (unsigned)out.pollsEmpty, (unsigned)out.pollsError);
      lastSats = s.sats;
    }

    if (graceDeadline && !out.fixValid && millis() > graceDeadline) {
      logf("[%s  ] fix crudo sin pasar el gate en %lu s de gracia: corto aqui",
           cfg.id, (unsigned long)POSTFIX_GRACE_S);
      if (out.note == "-") out.note = "fix crudo sin pasar el gate";
      else                 out.note += "; fix crudo sin pasar el gate";
      break;
    }
  }

  if (!out.fixRaw) {
    logf("[%s  ] sin fix crudo en %lu s (techo de la prueba)",
         cfg.id, (unsigned long)FIX_TIMEOUT_S);
    if (out.note == "-") out.note = "sin fix crudo";
    else                 out.note += "; sin fix crudo";
  }

  out.modeAfter = fieldAfter(atSend("AT+CGNSSMODE?", 3000, false), "+CGNSSMODE:");
  logf("[%s  ] CGNSSMODE con GNSS encendido: %s", cfg.id, out.modeAfter.c_str());

  gnssOff();
  logf("[%s  ] FIN", cfg.id);
  if (coolAfter) coolDown(cfg.id);
}

// =============================================================================
//  Resumen
// =============================================================================
static void printSummary() {
  bool anyGate = false, anyRaw = false;
  for (int i = 0; i < RUN_COUNT; i++) {
    if (!results[i].ran) continue;
    if (results[i].fixValid) anyGate = true;
    if (results[i].fixRaw)   anyRaw  = true;
  }

  Serial.println();
  Serial.println("=================================================================");
  Serial.println(" RESUMEN GNSS LAB 1.2 - desempate del sueno por DTR");
  Serial.println("=================================================================");
  if (skyOk) {
    Serial.printf(" cielo (baseline): fix crudo en %lu s, sats %d, hdop %.1f\r\n",
                  (unsigned long)(skyTtffMs / 1000), skySats, skyHdop);
  } else {
    Serial.println(" cielo (baseline): SIN FIX. El sitio no da para esta prueba.");
  }
  if (cmdSupport.length())  Serial.printf(" comandos de arranque: %s\r\n", cmdSupport.c_str());
  if (rechargesUsed)        Serial.printf(" recargas de NVRAM usadas: %d\r\n", rechargesUsed);
  if (abortReason.length()) Serial.printf(" ABORTADO: %s\r\n", abortReason.c_str());
  Serial.println("-----------------------------------------------------------------");

  Serial.printf("%-3s %-14s %-14s %-9s %7s %8s %8s %5s %5s %6s %5s %6s  %s\r\n",
                "run", "variable", "pre", "start", "off_s", "ttff_raw", "ttff_val",
                "sats", "hdop", "vacios", "err", "ready", "nota");

  for (int i = 0; i < RUN_COUNT; i++) {
    if (!results[i].ran) continue;
    RunResult &r = results[i];
    char raw[12], val[12], sats[8], hdop[8];
    if (r.fixRaw)   snprintf(raw, sizeof(raw), "%lus", (unsigned long)(r.ttffRawMs / 1000));
    else            snprintf(raw, sizeof(raw), "-");
    if (r.fixValid) snprintf(val, sizeof(val), "%lus", (unsigned long)(r.ttffValMs / 1000));
    else            snprintf(val, sizeof(val), "-");
    if (r.fixValid) snprintf(sats, sizeof(sats), "%d", r.sats);
    else            snprintf(sats, sizeof(sats), "-");
    if (r.hdop > 0)  snprintf(hdop, sizeof(hdop), "%.1f", r.hdop);
    else             snprintf(hdop, sizeof(hdop), "-");

    Serial.printf("%-3s %-14s %-14s %-9s %6lus %8s %8s %5s %5s %6u %5u %6s  %s\r\n",
                  RUNS[i].id, RUNS[i].label, preName(RUNS[i].pre), startName(RUNS[i].start),
                  (unsigned long)r.offTotalS, raw, val, sats, hdop,
                  (unsigned)r.pollsEmpty, (unsigned)r.pollsError,
                  r.ready ? "si" : "no", r.note.c_str());
  }

  Serial.println();
  Serial.println("=== CSV (pegar en el log de sesion) ===");
  Serial.println("run,variable,pre,start,off_s,ttff_raw_s,ttff_valid_s,sats,hdop,"
                 "polls_vacios,polls_error,polls_total,ready,mode_post,vbat,nota");
  for (int i = 0; i < RUN_COUNT; i++) {
    if (!results[i].ran) continue;
    RunResult &r = results[i];
    String note = r.note;
    note.replace(",", ";");
    Serial.printf("%s,%s,%s,%s,%lu,%ld,%ld,%d,%.1f,%u,%u,%u,%s,%s,%.2f,%s\r\n",
                  RUNS[i].id, RUNS[i].label, preName(RUNS[i].pre), startName(RUNS[i].start),
                  (unsigned long)r.offTotalS,
                  r.fixRaw   ? (long)(r.ttffRawMs / 1000) : -1L,
                  r.fixValid ? (long)(r.ttffValMs / 1000) : -1L,
                  r.fixValid ? r.sats : -1,
                  r.hdop,
                  (unsigned)r.pollsEmpty, (unsigned)r.pollsError, (unsigned)r.pollsTotal,
                  r.ready ? "si" : "no", r.modeAfter.c_str(), r.batV, note.c_str());
  }
  Serial.printf("P0,baseline_cielo,-,cgnsspwr,0,%ld,-1,%d,%.1f,0,0,0,no,-,0.00,%s\r\n",
                skyOk ? (long)(skyTtffMs / 1000) : -1L, skySats, skyHdop,
                skyOk ? "cielo suficiente" : "sin fix en el baseline");
  Serial.println("=== fin CSV ===");
  Serial.println();

  Serial.println("Como se lee:");
  Serial.println(" - T1 corto, T2 corto, T3 largo -> lo rompe el sueno por DTR, no el tiempo.");
  Serial.println("   Produccion: no dormir el modem en paradas cortas, o usar T8 (hot tras sueno).");
  Serial.println(" - T1 corto, T2 largo, T3 largo -> lo rompe el tiempo apagado. El sueno es");
  Serial.println("   inocente y F23 (ventana de gracia) cubre el caso; no hay 90 s que ganar.");
  Serial.println(" - T1, T2 y T3 los tres cortos -> el 120 s de R3 fue ruido de una sola muestra.");
  Serial.println(" - T4 o T5 por debajo de T1 -> hay comando de arranque asistido y vale oro:");
  Serial.println("   entra en F19 antes que cualquier otra cosa.");
  Serial.println(" - T4/T5 con nota 'no soportado' -> este firmware no los tiene, se cierra el tema.");
  Serial.println(" - ready=si en alguna corrida -> el URC READY si existe y hay que revisar F19.");
  if (anyRaw && !anyGate) {
    Serial.println();
    Serial.println(" AVISO: ninguna corrida paso el gate de calidad (tipico en interiores).");
    Serial.println(" Compara solo ttff_raw entre corridas; todas midieron en el mismo sitio.");
  }
  if (!anyRaw) {
    Serial.println();
    Serial.println(" AVISO: ninguna corrida consiguio fix. No concluyente: repite en exteriores.");
  }
  Serial.println();
  Serial.println("Este env NO es el tracker. Para volver a produccion:");
  Serial.println("  pio run -e tracker -t upload");
  Serial.println("=================================================================");
}

// =============================================================================
//  Arranque
// =============================================================================
static void banner() {
  int enabled = 0;
  for (int i = 0; i < RUN_COUNT; i++) if (RUNS[i].enabled) enabled++;

  Serial.println();
  Serial.println("=================================================================");
  Serial.println(" GPS Tracker Logan - GNSS LAB");
  Serial.println("=================================================================");
  Serial.printf(" version  : %s\r\n", LAB_VERSION);
  Serial.printf(" build    : %s %s\r\n", __DATE__, __TIME__);
  Serial.printf(" reset    : %s\r\n", resetReasonName());
  Serial.printf(" corridas : %d activas, techo %lu s de fix, %lu s de enfriado\r\n",
                enabled, (unsigned long)FIX_TIMEOUT_S, (unsigned long)GNSS_OFF_WAIT_S);
  Serial.println(" sondeo   : cada 1 s desde el instante del comando de arranque");
  Serial.println(" gate     : sats >= 5 y HDOP <= 2.5 (igual que main.cpp)");
  Serial.printf(" filtro   : P0 exige fix crudo en %lu s o aborta\r\n",
                (unsigned long)PREFLIGHT_TIMEOUT_S);
  Serial.println(" duracion : ~25-30 min tipico, hasta ~45 min en el peor caso");
  Serial.println("=================================================================");
  Serial.println("  P0 baseline_cielo  filtro de sitio y efemerides frescas");
  for (int i = 0; i < RUN_COUNT; i++) {
    Serial.printf("  %s %-15s pre=%-14s start=%-9s %s\r\n",
                  RUNS[i].id, RUNS[i].label, preName(RUNS[i].pre),
                  startName(RUNS[i].start), RUNS[i].enabled ? "" : "(desactivada)");
  }
  Serial.println("=================================================================");
}

// Solo pregunta si el comando existe. La sintaxis de test no arranca nada.
static void probeStartCommands() {
  struct { const char *test; const char *label; } probes[] = {
    { "AT+CGPSHOT=?",  "CGPSHOT"  },
    { "AT+CGPSWARM=?", "CGPSWARM" },
    { "AT+CGPSCOLD=?", "CGPSCOLD" },
  };
  for (unsigned i = 0; i < sizeof(probes) / sizeof(probes[0]); i++) {
    String r = atSend(probes[i].test, 3000, false);
    bool ok = r.indexOf("OK") >= 0;
    if (cmdSupport.length()) cmdSupport += " ";
    cmdSupport += String(probes[i].label) + "=" + (ok ? "si" : "no");
  }
  logf("[LAB ] soporte de comandos de arranque: %s", cmdSupport.c_str());
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < SERIAL_WAIT_MS) delay(50);
  delay(300);

  banner();

  logf("[LAB ] arranco en %lu s (manda cualquier tecla para empezar ya)",
       (unsigned long)START_COUNTDOWN_S);
  uint32_t cd = millis();
  while (millis() - cd < START_COUNTDOWN_S * 1000UL) {
    if (Serial.available()) { while (Serial.available()) Serial.read(); break; }
    delay(100);
  }

  // Pines igual que main.cpp: DTR bajo = modem despierto.
  pinMode(MODEM_DTR_PIN, OUTPUT);
  digitalWrite(MODEM_DTR_PIN, LOW);
  pinMode(BOARD_PWRKEY_PIN, OUTPUT);
  digitalWrite(BOARD_PWRKEY_PIN, LOW);
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);
  delay(200);

  bool alive = modemAnswers(2000);
  logf("[LAB ] estado inicial del modem: %s",
       alive ? "VIVO (el flasheo por USB no lo apago)"
             : "MUDO (arranque limpio, como debe ser)");
  if (!alive) {
    pwrkeyPulse();
    alive = waitModemAt(60000UL);
  }
  if (!alive) {
    logf("[LAB ] el modem no responde AT. Revisa alimentacion y polaridad de PWRKEY.");
    abortReason = "el modem nunca respondio AT";
    printSummary();
    labDone = true;
    return;
  }

  delay(1000);
  echoOff();
  atSend("AT+SIMCOMATI", 5000, false);
  atSend("AT+CSCLK?", 3000, false);
  atSend("AT+CGNSSPWR?", 3000, false);
  probeStartCommands();

  logf("[LAB ] dejo el GNSS apagado 20 s para partir de un estado conocido");
  gnssOff();
  delay(20000);

  // ---- P0: filtro de sitio y efemerides frescas ----------------------------
  if (!warmBaseline("P0")) {
    abortReason = "sin fix en el baseline: sitio insuficiente, repetir en exteriores";
    logf("[LAB ] %s", abortReason.c_str());
    printSummary();
    labDone = true;
    return;
  }

  // ---- corridas ------------------------------------------------------------
  for (int i = 0; i < RUN_COUNT; i++) {
    if (!RUNS[i].enabled) continue;

    bool anyLeft = false;
    for (int j = i + 1; j < RUN_COUNT; j++) if (RUNS[j].enabled) { anyLeft = true; break; }

    runOne(RUNS[i], results[i], anyLeft);

    if (results[i].fixRaw || !anyLeft) continue;

    // Sin fix crudo las efemerides quedan frias y la corrida siguiente no
    // seria comparable. Recargo con un baseline antes de continuar.
    if (rechargesUsed >= MAX_RECHARGES) {
      abortReason = "dos recargas ya usadas y sigue sin fijar: no concluyente";
      logf("[LAB ] %s", abortReason.c_str());
      break;
    }
    rechargesUsed++;
    logf("[LAB ] recarga de efemerides %d/%d antes de la corrida siguiente",
         rechargesUsed, MAX_RECHARGES);
    if (!warmBaseline("Pr")) {
      abortReason = "la recarga no consiguio fix: repetir en exteriores";
      logf("[LAB ] %s", abortReason.c_str());
      break;
    }
  }

  gnssOff();
  printSummary();
  labDone = true;
}

void loop() {
  static uint32_t last = 0;
  if (labDone && millis() - last > 120000UL) {
    last = millis();
    printSummary();
  }
  delay(500);
}
