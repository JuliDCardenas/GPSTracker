// =============================================================================
//  GPS Tracker Logan  ·  GNSS LAB  ·  src/gnss_lab.cpp  ·  env: gnss_lab
// =============================================================================
//  Banco automatizado para cerrar F19: mide el TTFF en varias condiciones
//  seguidas y senala que esta reiniciando el motor GNSS o apagando el modem.
//  Sustituye la prueba manual con at_passthrough (cronometro a mano).
//
//  CONDICIONES IGUALADAS CON main.cpp
//    - UART1 a 115200 en los mismos pines (RX 5 / TX 4).
//    - Encendido del modem identico a modemPowerOn(): DTR LOW, PWRKEY LOW,
//      100 ms, HIGH, 1000 ms, LOW.  OJO: at_passthrough.cpp tiene la polaridad
//      invertida (HIGH -> LOW -> HIGH y lo deja alto); aqui se usa la de
//      main.cpp, que es la de la secuencia oficial LilyGo.
//    - Encendido del GNSS identico a pmGnssOn(): CGDRT=1,1 + CGSETV=1,1 +
//      CGNSSPWR=1 (espera "+CGNSSPWR: READY") + CGNSSIPR=115200 + CGNSSMODE=15.
//    - Gate de calidad de main.cpp: sats >= 5 y HDOP <= 2.5. Se cronometran los
//      dos hitos por separado: primer fix crudo y primer fix que pasa el gate.
//      Si el crudo llega en 40 s y el gate en 3 min, el problema no es el NVRAM.
//
//  CONDICIONES QUE NO SE REPRODUCEN (a proposito)
//    - Sin LTE, sin MQTT, sin deep sleep: el laboratorio necesita el USB vivo.
//    - El hold de pines (gpio_hold_en / gpio_deep_sleep_hold_en) no se ejercita.
//      R3 prueba el sueno por DTR a nivel AT con el pin manejado normalmente.
//
//  ANTES DE FLASHEAR
//    1. Desconecta USB y saca la 18650 unos 10 s: el modem debe arrancar
//       apagado de verdad, si no el pulso de PWRKEY cae sobre modem vivo y
//       contamina R0.
//    2. Conecta USB, flashea y abre el monitor:
//         pio run -e gnss_lab -t upload
//         pio device monitor -e gnss_lab
//    3. Antena GNSS quieta y con el cielo mas abierto que puedas darle.
//       Duracion 30-55 min.
//
//  CORRER EN INTERIORES
//    P0 es el filtro: enciende el GNSS y exige un fix crudo en 240 s. Si no
//    lo consigue, el laboratorio aborta ahi mismo y te dice que lo repitas en
//    exteriores; no gasta los 40 min restantes en seis pruebas ciegas.
//    Si P0 pasa, el banco sirve en interiores pero solo para comparar los
//    ttff_raw entre corridas: el gate de calidad (sats >= 5, HDOP <= 2.5) casi
//    nunca pasa bajo techo, y por eso cada corrida corta 60 s despues del fix
//    crudo en vez de esperar el techo de 300 s.
//    Si una corrida se queda sin fix crudo, el banco recarga el NVRAM con otro
//    baseline antes de seguir (maximo dos veces) para que la corrida siguiente
//    arranque en igualdad de condiciones. A la tercera falla, aborta.
//
//  COMO SE LEE EL RESULTADO
//    R1 (control, sin tocar nada) deberia dar TTFF corto (~30 s). Si R1 tambien
//    tarda minutos, el NVRAM no retiene nada y F19 muere: la respuesta es F23
//    (ventana de gracia, dejar el GNSS encendido 10-20 min).
//    Si R1 es corto y otra corrida tarda minutos, esa corrida es la culpable:
//      R2 largo    -> lo rompe reiniciar el modem (restartModem / pulso).
//      R3 largo    -> lo rompe el sueno por DTR.
//      R4 largo    -> lo rompe reescribir CGNSSMODE=15 en cada boot. F19 = 3
//                     lineas y cuesta cero mA.
//      R5 sin AT   -> el pulso de PWRKEY sobre modem vivo lo apaga.
//    Suelo fisico: las subtramas GPS L1 C/A salen cada 30 s. Sin A-GNSS no hay
//    TTFF por debajo de ~30 s, no persigas mas que eso.
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

// ------------------------------------------------- gate de calidad de main.cpp
#define MIN_VALID_SATELLITES          5
#define MAX_VALID_HDOP                2.5f

// ------------------------------------------------------- parametros del banco
#define LAB_VERSION                   "gnss_lab 1.1"
#define FIX_TIMEOUT_S                 300UL   // techo por corrida
#define PREFLIGHT_TIMEOUT_S           240UL   // techo del baseline de cielo
#define POSTFIX_GRACE_S               60UL    // margen para pasar el gate
#define MAX_RECHARGES                 2       // baselines de rescate permitidos
#define POLL_PERIOD_MS                2000UL  // cadencia de CGNSSINFO
#define POLL_LOG_EVERY                5       // imprime 1 de cada N sondeos
#define GNSS_OFF_WAIT_S               150UL   // GNSS apagado entre corridas
#define DTR_SLEEP_S                   120UL   // duracion del sueno por DTR
#define START_COUNTDOWN_S             10UL
#define SERIAL_WAIT_MS                8000UL

HardwareSerial SerialAT(1);

// =============================================================================
//  Definicion de las corridas
// =============================================================================
enum Pre : uint8_t {
  PRE_NONE,           // nada: control
  PRE_CPOF_PWRKEY,    // AT+CPOF y reencendido por PWRKEY: arranque frio real
  PRE_CRESET,         // AT+CRESET
  PRE_DTR_SLEEP,      // CSCLK=1 + DTR alto + despertar
  PRE_PWRKEY_PULSE    // pulso de PWRKEY sobre modem vivo
};

enum Post : uint8_t {
  POST_NONE,          // solo CGNSSPWR=1
  POST_BAUD,          // + CGNSSIPR=115200
  POST_BAUD_MODE      // + CGNSSIPR=115200 + CGNSSMODE=15  (lo que hace el boot)
};

struct RunCfg {
  const char *id;
  const char *label;
  bool        enabled;
  Pre         pre;
  Post        post;
};

static const RunCfg RUNS[] = {
  { "R0", "cold_ref",     true,  PRE_CPOF_PWRKEY,  POST_BAUD_MODE },
  { "R1", "warm_ctrl",    true,  PRE_NONE,         POST_NONE      },
  { "R2", "creset",       true,  PRE_CRESET,       POST_NONE      },
  { "R3", "dtr_sleep",    true,  PRE_DTR_SLEEP,    POST_NONE      },
  { "R4", "cgnssmode15",  true,  PRE_NONE,         POST_BAUD_MODE },
  { "R5", "pwrkey_pulse", true,  PRE_PWRKEY_PULSE, POST_NONE      },
  // Desempate: si R4 sale largo, activa R6 para saber si el culpable es
  // CGNSSIPR (baud) o CGNSSMODE. Suma ~8 min.
  { "R6", "baud_only",    false, PRE_NONE,         POST_BAUD      },
};
static const int RUN_COUNT = sizeof(RUNS) / sizeof(RUNS[0]);

struct RunResult {
  bool     ran        = false;
  bool     fixRaw     = false;
  bool     fixValid   = false;
  uint32_t ttffRawMs  = 0;
  uint32_t ttffValMs  = 0;
  int      sats       = 0;
  float    hdop       = -1.0f;
  uint16_t pollsEmpty = 0;
  uint16_t pollsError = 0;
  uint16_t pollsTotal = 0;
  float    batV       = 0.0f;
  String   modeBefore = "-";
  String   modeAfter  = "-";
  String   note       = "-";
};

static RunResult results[RUN_COUNT];
static bool   labDone      = false;

// estado del cielo medido por el baseline
static bool     skyOk       = false;
static uint32_t skyTtffMs   = 0;
static int      skySats     = 0;
static float    skyHdop     = -1.0f;
static int      rechargesUsed = 0;
static String   abortReason = "";

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
    case PRE_CPOF_PWRKEY:  return "cpof+pwrkey";
    case PRE_CRESET:       return "creset";
    case PRE_DTR_SLEEP:    return "dtr_sleep";
    case PRE_PWRKEY_PULSE: return "pwrkey_pulse";
    default:               return "nada";
  }
}

static const char *postName(Post p) {
  switch (p) {
    case POST_BAUD:      return "baud";
    case POST_BAUD_MODE: return "baud+mode";
    default:             return "nada";
  }
}

static const char *resetReasonName() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  return "POWERON";
    case ESP_RST_EXT:      return "EXT";
    case ESP_RST_SW:       return "SW";
    case ESP_RST_PANIC:    return "PANIC";
    case ESP_RST_INT_WDT:  return "INT_WDT";
    case ESP_RST_TASK_WDT: return "TASK_WDT";
    case ESP_RST_WDT:      return "WDT";
    case ESP_RST_DEEPSLEEP:return "DEEPSLEEP";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_SDIO:     return "SDIO";
    default:               return "UNKNOWN";
  }
}

static float readBatV() {
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

// Apaga el modem de verdad por software y lo vuelve a encender.
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

// CSCLK=1 + DTR alto = el modem duerme. DTR bajo = despierta.
static String dtrSleepCycle() {
  logf("[MDM ] --- sueno por DTR: CSCLK=1 + DTR HIGH %lu s ---",
       (unsigned long)DTR_SLEEP_S);
  atOk("AT+CSCLK=1", 3000);
  digitalWrite(MODEM_DTR_PIN, HIGH);
  delay(5000);

  bool awakeWithDtrHigh = modemAnswers(1500);
  logf("[MDM ] con DTR alto el modem %s",
       awakeWithDtrHigh ? "SIGUE contestando (no durmio)" : "no contesta (durmio)");

  uint32_t t0 = millis();
  while (millis() - t0 < DTR_SLEEP_S * 1000UL) {
    delay(1000);
    uint32_t el = (millis() - t0) / 1000UL;
    if (el % 30 == 0) logf("[MDM ] dormido por DTR %lu/%lu s",
                           (unsigned long)el, (unsigned long)DTR_SLEEP_S);
  }

  digitalWrite(MODEM_DTR_PIN, LOW);
  delay(200);
  bool back = waitModemAt(20000UL);
  atOk("AT+CSCLK=0", 3000);
  if (!awakeWithDtrHigh && back) return "durmio y despertio por DTR";
  if (awakeWithDtrHigh)         return "no llego a dormir con DTR alto";
  return "no despertio por DTR";
}

static String pwrkeyOnLiveModem() {
  logf("[MDM ] --- pulso de PWRKEY sobre modem VIVO (sospechoso 3) ---");
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
}

// Devuelve el instante exacto del CGNSSPWR=1: la referencia del TTFF.
static uint32_t gnssOn(bool &readyOut) {
  atSend("AT+CGDRT=1,1", 3000, false);       // alimentacion de antena GNSS
  atSend("AT+CGSETV=1,1", 3000, false);

  uint32_t t0 = millis();
  String r = atSend("AT+CGNSSPWR=1", 6000, false);
  bool ready = r.indexOf("READY") >= 0;
  if (!ready) {
    String acc;
    uint32_t w0 = millis();
    while (millis() - w0 < 30000UL) {
      while (SerialAT.available()) acc += (char)SerialAT.read();
      if (acc.indexOf("READY") >= 0) { ready = true; break; }
      delay(20);
    }
    if (acc.length()) logf("[AT  ] < %s", oneLine(acc).c_str());
  }
  logf("[GNSS] CGNSSPWR=1 -> %s tras %lu ms",
       ready ? "READY" : "sin READY", (unsigned long)(millis() - t0));
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
//  Sirve de filtro en interiores y deja el NVRAM caliente y comparable.
// =============================================================================
static bool warmBaseline(const char *tag) {
  Serial.println();
  Serial.println("-----------------------------------------------------------------");
  logf("[%s  ] BASELINE DE CIELO: enciendo GNSS y espero fix crudo (max %lu s)",
       tag, (unsigned long)PREFLIGHT_TIMEOUT_S);
  Serial.println("-----------------------------------------------------------------");

  bool ready = false;
  uint32_t t0 = gnssOn(ready);

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
static void runOne(const RunCfg &cfg, RunResult &out) {
  Serial.println();
  Serial.println("-----------------------------------------------------------------");
  logf("[%s  ] INICIO  %s   pre=%s  post=%s",
       cfg.id, cfg.label, preName(cfg.pre), postName(cfg.post));
  Serial.println("-----------------------------------------------------------------");

  out.ran  = true;
  out.batV = readBatV();
  logf("[%s  ] Vbat aprox %.2f V", cfg.id, out.batV);

  // ---- variable bajo prueba -------------------------------------------------
  switch (cfg.pre) {
    case PRE_CPOF_PWRKEY:  out.note = modemColdCycle();    break;
    case PRE_CRESET:       out.note = modemReset();        break;
    case PRE_DTR_SLEEP:    out.note = dtrSleepCycle();     break;
    case PRE_PWRKEY_PULSE: out.note = pwrkeyOnLiveModem(); break;
    default:               out.note = "-";                 break;
  }

  // ---- que recuerda el NVRAM antes de encender ------------------------------
  out.modeBefore = fieldAfter(atSend("AT+CGNSSMODE?", 3000, false), "+CGNSSMODE:");
  logf("[%s  ] CGNSSMODE antes de encender: %s", cfg.id, out.modeBefore.c_str());

  // ---- encendido del GNSS, igual que pmGnssOn() -----------------------------
  bool ready = false;
  uint32_t t0 = gnssOn(ready);
  if (!ready && out.note == "-") out.note = "CGNSSPWR=1 sin READY";

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

    // En interiores el gate casi nunca pasa: no gastes el techo entero.
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
  coolDown(cfg.id);
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
  Serial.println(" RESUMEN GNSS LAB");
  Serial.println("=================================================================");
  if (skyOk) {
    Serial.printf(" cielo (baseline): fix crudo en %lu s, sats %d, hdop %.1f\r\n",
                  (unsigned long)(skyTtffMs / 1000), skySats, skyHdop);
  } else {
    Serial.println(" cielo (baseline): SIN FIX. El sitio no da para esta prueba.");
  }
  if (rechargesUsed) Serial.printf(" recargas de NVRAM usadas: %d\r\n", rechargesUsed);
  if (abortReason.length()) Serial.printf(" ABORTADO: %s\r\n", abortReason.c_str());
  Serial.println("-----------------------------------------------------------------");

  Serial.printf("%-3s %-13s %-13s %-10s %8s %8s %5s %5s %6s %5s %8s %9s  %s\r\n",
                "run", "variable", "pre", "post", "ttff_raw", "ttff_val",
                "sats", "hdop", "vacios", "err", "mode_pre", "mode_post", "nota");

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

    Serial.printf("%-3s %-13s %-13s %-10s %8s %8s %5s %5s %6u %5u %8s %9s  %s\r\n",
                  RUNS[i].id, RUNS[i].label, preName(RUNS[i].pre), postName(RUNS[i].post),
                  raw, val, sats, hdop,
                  (unsigned)r.pollsEmpty, (unsigned)r.pollsError,
                  r.modeBefore.c_str(), r.modeAfter.c_str(), r.note.c_str());
  }

  Serial.println();
  Serial.println("=== CSV (pegar en el log de sesion) ===");
  Serial.println("run,variable,pre,post,ttff_raw_s,ttff_valid_s,sats,hdop,"
                 "polls_vacios,polls_error,polls_total,mode_pre,mode_post,vbat,nota");
  for (int i = 0; i < RUN_COUNT; i++) {
    if (!results[i].ran) continue;
    RunResult &r = results[i];
    String note = r.note;
    note.replace(",", ";");
    Serial.printf("%s,%s,%s,%s,%ld,%ld,%d,%.1f,%u,%u,%u,%s,%s,%.2f,%s\r\n",
                  RUNS[i].id, RUNS[i].label, preName(RUNS[i].pre), postName(RUNS[i].post),
                  r.fixRaw   ? (long)(r.ttffRawMs / 1000) : -1L,
                  r.fixValid ? (long)(r.ttffValMs / 1000) : -1L,
                  r.fixValid ? r.sats : -1,
                  r.hdop,
                  (unsigned)r.pollsEmpty, (unsigned)r.pollsError, (unsigned)r.pollsTotal,
                  r.modeBefore.c_str(), r.modeAfter.c_str(), r.batV, note.c_str());
  }
  Serial.printf("P0,baseline_cielo,-,-,%ld,-1,%d,%.1f,0,0,0,-,-,0.00,%s\r\n",
                skyOk ? (long)(skyTtffMs / 1000) : -1L, skySats, skyHdop,
                skyOk ? "cielo suficiente" : "sin fix en el baseline");
  Serial.println("=== fin CSV ===");
  Serial.println();

  Serial.println("Lectura rapida:");
  Serial.println(" - R1 corto y otra corrida larga -> esa es la culpable, F19 = no hacer eso.");
  Serial.println(" - R1 tambien largo -> el NVRAM no retiene: F19 muere, va F23 (ventana de gracia).");
  Serial.println(" - ttff_raw corto y ttff_valid largo -> el problema es el gate, no el NVRAM.");
  Serial.println(" - R5 con nota 'el pulso APAGO el modem' -> hay que guardar rtcModemAlive de verdad.");
  if (anyRaw && !anyGate) {
    Serial.println();
    Serial.println(" AVISO: ninguna corrida paso el gate de calidad (tipico en interiores).");
    Serial.println(" Compara solo la columna ttff_raw entre corridas; el veredicto sigue siendo");
    Serial.println(" valido para F19 porque todas las corridas midieron en el mismo sitio.");
  }
  if (!anyRaw) {
    Serial.println();
    Serial.println(" AVISO: ninguna corrida consiguio fix. Prueba no concluyente: repite en");
    Serial.println(" exteriores con la antena al cielo abierto.");
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
  unsigned long worstMin =
    (unsigned long)(((PREFLIGHT_TIMEOUT_S + GNSS_OFF_WAIT_S) +
                     enabled * (FIX_TIMEOUT_S + GNSS_OFF_WAIT_S + 90UL)) / 60UL);

  Serial.println();
  Serial.println("=================================================================");
  Serial.println(" GPS Tracker Logan - GNSS LAB");
  Serial.println("=================================================================");
  Serial.printf(" version  : %s\r\n", LAB_VERSION);
  Serial.printf(" build    : %s %s\r\n", __DATE__, __TIME__);
  Serial.printf(" reset    : %s\r\n", resetReasonName());
  Serial.printf(" corridas : %d activas, techo %lu s de fix, %lu s de enfriado\r\n",
                enabled, (unsigned long)FIX_TIMEOUT_S, (unsigned long)GNSS_OFF_WAIT_S);
  Serial.printf(" duracion : hasta ~%lu min en el peor caso\r\n", worstMin);
  Serial.println(" gate     : sats >= 5 y HDOP <= 2.5 (igual que main.cpp)");
  Serial.printf(" filtro   : P0 exige fix crudo en %lu s o aborta\r\n",
                (unsigned long)PREFLIGHT_TIMEOUT_S);
  Serial.println("=================================================================");
  Serial.println("  P0 baseline_cielo  filtro y NVRAM caliente");
  for (int i = 0; i < RUN_COUNT; i++) {
    Serial.printf("  %s %-13s pre=%-13s post=%-10s %s\r\n",
                  RUNS[i].id, RUNS[i].label, preName(RUNS[i].pre),
                  postName(RUNS[i].post), RUNS[i].enabled ? "" : "(desactivada)");
  }
  Serial.println("=================================================================");
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
  atSend("AT+CGNSSMODE?", 3000, false);
  atSend("AT+CGNSSCOLD=?", 3000, false);   // solo existencia, no lo ejecuta

  logf("[LAB ] dejo el GNSS apagado 20 s para partir de un estado conocido");
  gnssOff();
  delay(20000);

  // ---- P0: filtro de sitio -------------------------------------------------
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
    runOne(RUNS[i], results[i]);

    if (results[i].fixRaw) continue;

    // Sin fix crudo el NVRAM queda frio y la corrida siguiente no seria
    // comparable. Recargo con un baseline antes de continuar.
    bool anyLeft = false;
    for (int j = i + 1; j < RUN_COUNT; j++) if (RUNS[j].enabled) { anyLeft = true; break; }
    if (!anyLeft) break;

    if (rechargesUsed >= MAX_RECHARGES) {
      abortReason = "dos recargas de NVRAM ya usadas y sigue sin fijar: no concluyente";
      logf("[LAB ] %s", abortReason.c_str());
      break;
    }
    rechargesUsed++;
    logf("[LAB ] recarga de NVRAM %d/%d antes de la corrida siguiente",
         rechargesUsed, MAX_RECHARGES);
    if (!warmBaseline("Pr")) {
      abortReason = "la recarga de NVRAM no consiguio fix: repetir en exteriores";
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
