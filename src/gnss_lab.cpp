// =============================================================================
//  GPS Tracker Logan  ·  GNSS LAB v1.4  ·  src/gnss_lab.cpp  ·  env: gnss_lab
// =============================================================================
//  TERCERA PASADA. La v1.3 se corto en el arranque: la red no subio y sin red
//  T9 (A-GNSS) no mide nada. Dos arreglos, y el segundo vale mas que el primero.
//
//  1. FALTABA AT+NETOPEN. Esto mato a la v1.3.
//     El log decia:
//        [AT ] > AT+IPADDR
//        [AT ] < | +IP ERROR: Network not opened | ERROR |
//     y despues de un AT+CGACT=1,1 seguia igual. La causa es que SIMCOM tiene
//     DOS capas separadas y la v1.3 solo levantaba la primera:
//        - contexto PDP        -> AT+CGACT=1,1   (capa 3GPP)
//        - servicio de sockets -> AT+NETOPEN     (pila TCP/IP propia de SIMCOM)
//     AT+IPADDR pregunta por la SEGUNDA. Sin NETOPEN siempre contesta
//     "Network not opened", tenga o no tenga contexto activo. Manual SIM767XX
//     V1.02, 13.2.1 (NETOPEN) y 13.2.7 (IPADDR).
//     Secuencia buena:
//        AT+NETOPEN   -> OK   y despues el URC  +NETOPEN: 0
//        AT+IPADDR    -> +IPADDR: 10.x.x.x
//     NETOPEN responde OK de inmediato y suelta el resultado real como URC
//     hasta 120 s despues, asi que hay que esperarlo aparte. Si ya estaba
//     abierto contesta "+IP ERROR: Network is already opened", que tambien
//     sirve.
//     Lo bueno del log de la v1.3: +CSQ: 24,0 y +CEREG: 0,5 en cero segundos.
//     La SIM, la antena LTE y la cobertura estan perfectas. Solo faltaba abrir.
//
//  2. GUARDIA DE TRAMA RANCIA. Este es el hallazgo de verdad.
//     En la corrida de la v1.2, T4 reporto fix a los 0 s tras 151 s con el
//     motor apagado, en su UNICO sondeo, con sats=19 y hdop=2.0. La ultima
//     lectura de T3, 153 s antes, fue sats=19 hdop=2.0. Identicas. Y sus dos
//     gemelas de la misma condicion (T1 con 22 s, T5 con 26 s) no hicieron eso.
//     Lo mas probable es que +CGNSSINFO devolviera la ultima solucion guardada
//     en vez de una nueva.
//     Es la TERCERA variante de la trama fantasma y es la peligrosa. Las dos
//     anteriores traian sats=0 y hdop=-1, y el gate las mataba. Esta trae
//     numeros impecables y pasa el gate sin despeinarse. En produccion eso es
//     publicar una posicion de hace dos minutos y medio como si fuera actual:
//     a 60 km/h, 2,5 km de mentira.
//     Lo unico que la delata es la fecha y la hora que vienen DENTRO de la
//     propia trama (campos 9 y 10 de +CGNSSINFO), que nunca miramos.
//     Aqui se guarda la marca de tiempo del ultimo fix aceptado y se rechaza
//     cualquier trama que llegue con esa misma marca: no es nueva, es la de
//     antes. Se cuenta en la columna 'rancias' y el cronometro sigue corriendo.
//     Si el guardia no dispara nunca, T4 fue una readquisicion genuina y
//     tambien es buena noticia. En los dos casos salimos ganando.
//
//  LO QUE YA QUEDO CONTESTADO (corridas del 2026-08-22)
//    - El tiempo con el GNSS apagado NO alarga el TTFF: 279 s apagado fijo en
//      5 s; 150 s apagado fijo en 22 s. Al reves de lo esperado. Descartado.
//    - El sueno por DTR cuesta ~15 s, no los 90 que temiamos. Se sigue
//      durmiendo el modem: es de donde sale el ahorro. F23 con ventana de
//      gracia de 150 s lo cubre de sobra.
//    - Los 120 s de R3 en la v1.1 eran el CIELO, no el DTR: el baseline de ese
//      dia fue 124 s (bajo techo) contra 17 s del dia siguiente. Por eso P0
//      manda y ninguna conclusion vale sin el.
//    - Repetibilidad del control: T1=22 s y T5=26 s en la misma condicion. El
//      piso de ruido es +/- 4 s con cielo bueno.
//    - CRESET no borra nada caro. El pulso de PWRKEY sobre modem vivo no lo
//      tumba. CGNSSMODE es SAVE-persistente y solo se puede leer con el motor
//      encendido. El URC +CGNSSPWR: READY! no existe en este firmware.
//
//  POR QUE EL EMPUJON VA DESPUES DEL ENCENDIDO
//    Las fichas de CGPSHOT, CGPSWARM y CGPSCOLD (21.2.3 a 21.2.5 del manual
//    SIM767XX V1.02) tienen UNA sola fila, Execution Command: no admiten la
//    forma =? ni la forma ?. Por eso el sondeo de la v1.2 dio ERROR y por eso
//    no probaba nada. Y empiezan con la nota "This command is valid after the
//    GNSS power on!", asi que van DESPUES de CGNSSPWR=1, no en su lugar. La
//    v1.2 los mandaba en vez del encendido, siguiendo un application note de
//    la serie SIM82XX, que es otra familia. De ahi los ERROR de T4 y T5.
//    El cronometro sigue arrancando en CGNSSPWR=1 para que todo sea comparable
//    contra T1; el empujon entra un par de segundos despues y se anota a que
//    milisegundo del encendido se mando.
//
//  POR QUE T5 (WARM) ES CONTROL Y VA DE ULTIMO
//    Por definicion, warm start conserva almanaque, hora y posicion pero
//    DESCARTA las efemerides. Partiendo de un estado caliente tiene que salir
//    mas lento, y encima deja frio al motor para lo que venga detras.
//    Entonces no es candidato: es control positivo. Si T5 sale claramente mas
//    lento que T1, queda probado que este firmware obedece la familia CGPSxxx
//    y solo entonces un T4 igual a T1 se puede leer como "ya estabamos
//    calientes" en vez de "el comando se ignora". Va de ultimo porque envenena.
//
//  QUE SE MIDE
//    T1  warm_ctrl   CGNSSPWR=1 y nada mas.                    Referencia.
//    T4  hot_cmd     CGNSSPWR=1 y despues AT+CGPSHOT.          El candidato.
//    T9  agps        CGNSSPWR=1 y despues AT+CAGPS.            El de 24 h.
//    T10 gnss_sleep  el motor no se apaga: CGNSSSLEEP 150 s y
//                    despues CGNSSWAKEUP.                      El de parqueo.
//    T5  warm_cmd    CGNSSPWR=1 y despues AT+CGPSWARM.         Control positivo.
//
//    T9 existe porque el corte de energia borra las efemerides SIEMPRE
//    (confirmado por LilyGO en el issue 453: viven en RAM alimentada, no en
//    flash). En el pulso de 24 h el frio es inevitable y bajar asistencia por
//    LTE es la unica salida. Se compara contra los 117 s de R0, no contra T1.
//    SIMCOM le confirmo a lewisxhe que A-GNSS si funciona en el SIM7670G
//    (issue 117 de T-SIM7600X) y que el manual que circulaba estaba mal.
//
//    T10 hace lo mismo que hace el tracker al estacionar, pero en vez de
//    cortarle la energia al motor lo duerme. Es la corrida donde el guardia de
//    trama rancia importa mas: despertar y que reporte fix instantaneo es
//    justo el escenario donde una trama vieja nos haria rediseniar el
//    estacionado por nada.
//
//  CONDICIONES IGUALADAS CON main.cpp
//    - UART1 a 115200 en los mismos pines (RX 5 / TX 4).
//    - Encendido identico a modemPowerOn(): DTR LOW, PWRKEY LOW, 100 ms, HIGH,
//      1000 ms, LOW.  OJO: at_passthrough.cpp tiene la polaridad invertida.
//    - Gate de calidad: sats >= 5 y HDOP <= 2.5.
//    - Bateria con warmup de 8 muestras descartadas, como ADC_WARMUP_READS.
//    - Sin espera del URC READY: 2 s de cortesia y a sondear cada segundo.
//
//  ANTES DE FLASHEAR
//    1. Desconecta USB y saca la 18650 unos 10 s.
//    2. pio run -e gnss_lab -t upload
//       pio device monitor -e gnss_lab
//    3. Antena quieta, mismo sitio que la corrida anterior si quieres comparar.
//    Duracion tipica 20-25 min; techo ~40 min.
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
#define LAB_VERSION                   "gnss_lab 1.4"
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
#define PRESLEEP_FIX_S                120UL   // techo del fix previo a CGNSSSLEEP
#define NET_TIMEOUT_S                 90UL    // techo del registro en red
#define NETOPEN_TIMEOUT_MS            45000UL // techo del URC +NETOPEN
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
  PRE_GNSS_SLEEP,       // fix, CGNSSSLEEP y esperar: el motor NO se apaga
  PRE_CPOF_PWRKEY,      // apagado real del modem: arranque frio de referencia
  PRE_CRESET,           // AT+CRESET
  PRE_PWRKEY_PULSE      // pulso de PWRKEY sobre modem vivo
};

// El arranque siempre es CGNSSPWR=1 y el cronometro parte de ahi. Lo que
// cambia es el EMPUJON que se manda encima, un par de segundos despues, con el
// motor ya encendido, que es lo que exige el manual.
enum Start : uint8_t {
  START_PWR,     // nada mas                      (identico a pmGnssOn)
  START_HOT,     // + AT+CGPSHOT                  candidato
  START_WARM,    // + AT+CGPSWARM                 control positivo, destructivo
  START_COLD,    // + AT+CGPSCOLD                 control negativo, destructivo
  START_AGPS,    // + AT+CAGPS  (o AT+CGNSSAGPS)  asistencia por LTE
  START_WAKEUP   // el motor esta dormido: AT+CGNSSWAKEUP
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
  // --- referencia -----------------------------------------------------------
  { "T1",  "warm_ctrl",   true,  PRE_NONE,            START_PWR,    POST_NONE },
  // --- el candidato ---------------------------------------------------------
  { "T4",  "hot_cmd",     true,  PRE_NONE,            START_HOT,    POST_NONE },
  // --- el que resuelve el escenario de 24 h estacionado ---------------------
  { "T9",  "agps",        true,  PRE_NONE,            START_AGPS,   POST_NONE },
  // --- el que puede rediseniar el estacionado -------------------------------
  { "T10", "gnss_sleep",  true,  PRE_GNSS_SLEEP,      START_WAKEUP, POST_NONE },
  // --- control positivo: DEBE salir lento. Va de ultimo porque envenena. -----
  { "T5",  "warm_cmd",    true,  PRE_NONE,            START_WARM,   POST_NONE },

  // --- segunda vuelta, activar solo si hace falta ---------------------------
  { "T11", "cold_cmd",    false, PRE_NONE,            START_COLD,   POST_NONE },
  { "T8",  "hot_tras_sueno", false, PRE_DTR_SLEEP,    START_HOT,    POST_NONE },
  { "T6",  "sleep_short", false, PRE_DTR_SLEEP_SHORT, START_PWR,    POST_NONE },
  { "T7",  "csclk_awake", false, PRE_CSCLK_AWAKE,     START_PWR,    POST_NONE },
  // --- ya respondidas en las corridas del 2026-08-22 ------------------------
  { "T2",  "long_off",    false, PRE_LONG_OFF,        START_PWR,    POST_NONE },
  { "T3",  "dtr_sleep",   false, PRE_DTR_SLEEP,       START_PWR,    POST_NONE },
  { "R0",  "cold_ref",    false, PRE_CPOF_PWRKEY,     START_PWR,    POST_BAUD_MODE },
  { "R2",  "creset",      false, PRE_CRESET,          START_PWR,    POST_NONE },
  { "R4",  "cgnssmode15", false, PRE_NONE,            START_PWR,    POST_BAUD_MODE },
  { "R5",  "pwrkey_live", false, PRE_PWRKEY_PULSE,    START_PWR,    POST_NONE },
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
  uint16_t pollsStale = 0;   // tramas con la marca de tiempo del fix anterior
  uint16_t pollsTotal = 0;
  float    batV       = 0.0f;
  String   modeAfter  = "-";
  String   stamp      = "-";  // fecha/hora que traia la trama aceptada
  String   note       = "-";
};

static RunResult results[RUN_COUNT];
static bool      labDone = false;

// estado del cielo medido por el baseline
static bool     skyOk         = false;
static uint32_t skyTtffMs     = 0;
static int      skySats       = 0;
static float    skyHdop       = -1.0f;
static String   skyStamp      = "-";
static int      rechargesUsed = 0;
static String   abortReason   = "";
static String   cmdSupport    = "(no probado)";
static uint32_t gnssOffSinceMs = 0;   // cuando se apago o durmio el GNSS
static bool     netOk         = false;
static String   netIp         = "-";

// Guardia de trama rancia: marca de tiempo del ultimo fix REAL que vimos.
// Cualquier trama que llegue con esta misma marca no es nueva.
static String   lastFixStamp  = "";
static uint16_t staleTotal    = 0;
static bool     stampAvail    = true;   // el firmware llena fecha/hora?

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
    case PRE_GNSS_SLEEP:      return "gnss_sleep";
    case PRE_CPOF_PWRKEY:     return "cpof+pwrkey";
    case PRE_CRESET:          return "creset";
    case PRE_PWRKEY_PULSE:    return "pwrkey_pulse";
    default:                  return "nada";
  }
}

static const char *startName(Start s) {
  switch (s) {
    case START_HOT:    return "hot";
    case START_WARM:   return "warm";
    case START_COLD:   return "cold";
    case START_AGPS:   return "agps";
    case START_WAKEUP: return "wakeup";
    default:           return "pwr";
  }
}

static const char *boostCmd(Start s) {
  switch (s) {
    case START_HOT:  return "AT+CGPSHOT";
    case START_WARM: return "AT+CGPSWARM";
    case START_COLD: return "AT+CGPSCOLD";
    case START_AGPS: return "AT+CAGPS";
    default:         return "";
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

// Espera un URC concreto que llega DESPUES del OK. NETOPEN funciona asi.
static String waitUrc(const char *tag, uint32_t timeoutMs) {
  String   acc;
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    while (SerialAT.available()) acc += (char)SerialAT.read();
    if (acc.indexOf(tag) >= 0) break;
    delay(50);
  }
  if (acc.length()) {
    logf("[AT  ] < %s   (URC tras %lu ms)", oneLine(acc).c_str(),
         (unsigned long)(millis() - t0));
  } else {
    logf("[AT  ] sin URC %s en %lu ms", tag, (unsigned long)(millis() - t0));
  }
  return acc;
}

// =============================================================================
//  Red: se levanta UNA vez al arrancar, para que A-GNSS tenga por donde bajar
//  la asistencia y para que ese tiempo no se le sume al TTFF de ninguna corrida.
//
//  Dos capas, y la v1.3 solo levantaba la primera:
//    contexto PDP        -> AT+CGACT=1,1   (capa 3GPP)
//    servicio de sockets -> AT+NETOPEN     (pila TCP/IP de SIMCOM)
//  AT+IPADDR pregunta por la segunda.
// =============================================================================
static bool netUp() {
  Serial.println();
  logf("[NET ] --- levantando la red (techo %lu s) ---", (unsigned long)NET_TIMEOUT_S);
  atSend("AT+CPIN?", 5000, false);
  atSend("AT+CSQ", 3000, false);

  uint32_t t0  = millis();
  bool     reg = false;
  while (millis() - t0 < NET_TIMEOUT_S * 1000UL) {
    String r = atSend("AT+CEREG?", 3000, false);
    String f = fieldAfter(r, "+CEREG:");
    // +CEREG: <n>,<stat>   stat 1 = registrado, 5 = roaming
    if (f.endsWith(",1") || f.endsWith(",5") ||
        f.indexOf(",1,") >= 0 || f.indexOf(",5,") >= 0) { reg = true; break; }
    delay(3000);
  }
  logf("[NET ] registro %s tras %lu s", reg ? "OK" : "FALLIDO",
       (unsigned long)((millis() - t0) / 1000));
  if (!reg) { netIp = "sin registro"; return false; }

  // Informativo: que APN quedo y si el contexto ya estaba activo.
  atSend("AT+CCLK?", 3000, false);      // hora de red: sirve para leer las marcas
  atSend("AT+CGDCONT?", 5000, false);
  atSend("AT+CGACT?", 5000, false);

  // ---- el paso que faltaba en la v1.3 --------------------------------------
  String ro = atSend("AT+NETOPEN", 20000, false);
  bool opened = ro.indexOf("+NETOPEN: 0") >= 0 ||
                ro.indexOf("already opened") >= 0;
  if (!opened && ro.indexOf("OK") >= 0) {
    // NETOPEN contesta OK de una y suelta el resultado real como URC despues.
    String urc = waitUrc("+NETOPEN:", NETOPEN_TIMEOUT_MS);
    opened = urc.indexOf("+NETOPEN: 0") >= 0;
  }
  logf("[NET ] servicio de sockets: %s", opened ? "ABIERTO" : "NO abrio");

  String r = atSend("AT+IPADDR", 8000, false);
  netIp = fieldAfter(r, "+IPADDR:");

  if (netIp.indexOf('.') < 0) {
    logf("[NET ] sin IP: activo el contexto PDP y reintento NETOPEN");
    atSend("AT+CGACT=1,1", 15000, false);
    delay(2000);
    String ro2 = atSend("AT+NETOPEN", 20000, false);
    if (ro2.indexOf("+NETOPEN:") < 0 && ro2.indexOf("already opened") < 0) {
      waitUrc("+NETOPEN:", NETOPEN_TIMEOUT_MS);
    }
    delay(1000);
    r = atSend("AT+IPADDR", 8000, false);
    netIp = fieldAfter(r, "+IPADDR:");
  }

  bool ok = netIp.indexOf('.') >= 0;
  logf("[NET ] IP: %s -> A-GNSS %s", netIp.c_str(),
       ok ? "tiene por donde bajar" : "NO va a poder bajar nada (T9 no sera valida)");
  return ok;
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
  bool   hasTag = false;
  bool   hasFix = false;
  int    sats   = 0;
  float  hdop   = -1.0f;
  String stamp  = "";   // fecha/hora que trae la propia trama: ddmmyy/hhmmss.s
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
  // Campos 9 y 10: fecha y hora UTC. Es lo unico que distingue una medida
  // nueva de la ultima solucion guardada.
  if (f[9].length() > 0 || f[10].length() > 0) s.stamp = f[9] + "/" + f[10];
  return s;
}

// Devuelve true si esta trama trae la misma marca de tiempo que el ultimo fix
// aceptado: entonces no es una medida nueva, es la de antes.
static bool isStale(const GnssSample &s) {
  if (!s.hasFix) return false;
  if (s.stamp.length() < 2) return false;      // sin marca no se puede juzgar
  if (lastFixStamp.length() < 2) return false; // no hay con que comparar
  return s.stamp == lastFixStamp;
}

static void notePriorFix(const GnssSample &s) {
  if (s.hasFix && s.stamp.length() >= 2) lastFixStamp = s.stamp;
}

static void gnssOff() {
  atSend("AT+CGNSSPWR=0", 5000, false);
  gnssOffSinceMs = millis();
}

static void gnssPowerOnRaw() {
  atSend("AT+CGDRT=1,1", 3000, false);       // alimentacion de antena GNSS
  atSend("AT+CGSETV=1,1", 3000, false);
  atSend("AT+CGNSSPWR=1", 8000, false);
}

// Pre de T10. Reproduce lo que hace el tracker de verdad al estacionar: tenia
// un fix, deja de necesitarlo y en vez de cortarle la energia al motor lo
// duerme. Si CGNSSSLEEP no existe, cae a CGNSSPWR=0 y la corrida se vuelve una
// copia de T1 con la nota puesta.
static String gnssSleepCycle() {
  logf("[GNSS] --- pre: enciendo el motor y busco fix antes de dormirlo ---");
  gnssPowerOnRaw();

  uint32_t t0  = millis();
  bool     got = false;
  uint32_t nextPoll = millis() + READY_WAIT_MS;
  while (millis() - t0 < PRESLEEP_FIX_S * 1000UL) {
    if (millis() < nextPoll) { delay(20); continue; }
    nextPoll = millis() + POLL_PERIOD_MS;
    GnssSample s = gnssPoll();
    if (s.hasFix && !isStale(s)) {
      got = true;
      notePriorFix(s);   // esta marca es la que T10 NO puede volver a ver
      logf("[GNSS] pre-sueno: fix con marca %s", s.stamp.c_str());
      break;
    }
    uint32_t el = (millis() - t0) / 1000UL;
    if (el % 20 == 0 && el > 0) {
      logf("[GNSS] pre-sueno t=%lus sats=%d hdop=%.1f (sin fix aun)",
           (unsigned long)el, s.sats, s.hdop);
    }
  }
  logf("[GNSS] pre-sueno: %s tras %lu s", got ? "FIX conseguido" : "SIN fix",
       (unsigned long)((millis() - t0) / 1000));

  String r     = atSend("AT+CGNSSSLEEP", 5000, false);
  bool   slept = r.indexOf("OK") >= 0;
  String note;
  if (slept) {
    gnssOffSinceMs = millis();
    note = got ? "motor dormido con CGNSSSLEEP tras fix"
               : "motor dormido con CGNSSSLEEP sin fix previo";
    logf("[GNSS] CGNSSSLEEP aceptado: el motor NO se apago, solo duerme");
  } else {
    note = "CGNSSSLEEP no soportado; apagado con CGNSSPWR=0";
    logf("[GNSS] %s", note.c_str());
    gnssOff();
  }

  logf("[GNSS] espero %lu s con el motor %s", (unsigned long)GNSS_OFF_WAIT_S,
       slept ? "dormido" : "apagado");
  uint32_t w0 = millis();
  while (millis() - w0 < GNSS_OFF_WAIT_S * 1000UL) {
    delay(1000);
    uint32_t el = (millis() - w0) / 1000UL;
    if (el % 30 == 0 && el > 0) {
      logf("[GNSS] %lu/%lu s", (unsigned long)el, (unsigned long)GNSS_OFF_WAIT_S);
    }
  }
  return note;
}

// Arranca el motor y devuelve el instante que sirve de cero para el TTFF.
// El cronometro SIEMPRE parte de CGNSSPWR=1 (o de CGNSSWAKEUP en T10) para que
// todas las corridas se puedan comparar contra T1. El comando de arranque
// forzado va DESPUES, con el motor ya encendido, que es lo que pide el manual.
static uint32_t gnssStart(Start st, bool &readyOut, String &noteOut) {
  readyOut = false;

  // ---- T10: el motor no esta apagado, esta dormido -------------------------
  if (st == START_WAKEUP) {
    uint32_t t0 = millis();
    String   r  = atSend("AT+CGNSSWAKEUP", 8000, false);
    if (r.indexOf("OK") >= 0) {
      noteOut = "CGNSSWAKEUP OK";
      logf("[GNSS] motor despertado sin volver a encenderlo");
    } else {
      noteOut = "CGNSSWAKEUP sin OK; medido con CGNSSPWR=1";
      logf("[GNSS] %s", noteOut.c_str());
      atSend("AT+CGDRT=1,1", 3000, false);
      atSend("AT+CGSETV=1,1", 3000, false);
      t0 = millis();
      atSend("AT+CGNSSPWR=1", 8000, false);
    }
    return t0;
  }

  // ---- camino normal: identico a pmGnssOn ----------------------------------
  atSend("AT+CGDRT=1,1", 3000, false);
  atSend("AT+CGSETV=1,1", 3000, false);

  uint32_t t0 = millis();
  String   r  = atSend("AT+CGNSSPWR=1", 8000, false);
  bool     ok = r.indexOf("OK") >= 0;
  bool  ready = r.indexOf("READY") >= 0;

  // Ventana corta solo para saber si algun dia el URC aparece. La v1.1 perdia
  // 30 s aqui en cada encendido, que es la raiz de F19.
  if (!ready) {
    String   acc;
    uint32_t w0 = millis();
    while (millis() - w0 < READY_WAIT_MS) {
      while (SerialAT.available()) acc += (char)SerialAT.read();
      if (acc.indexOf("READY") >= 0) { ready = true; break; }
      delay(10);
    }
    if (acc.length()) logf("[AT  ] < %s", oneLine(acc).c_str());
  }
  logf("[GNSS] AT+CGNSSPWR=1 -> %s%s tras %lu ms", ok ? "OK" : "SIN OK",
       ready ? " + READY" : " (sin READY, normal en este firmware)",
       (unsigned long)(millis() - t0));
  readyOut = ready;

  if (st == START_PWR) { noteOut = "-"; return t0; }

  // ---- empujon con el motor ya encendido -----------------------------------
  const char *cmd = boostCmd(st);
  uint32_t    off = millis() - t0;

  if (st == START_AGPS && !netOk) {
    noteOut = "sin red: A-GNSS no es valido en esta corrida";
    logf("[GNSS] %s", noteOut.c_str());
    return t0;
  }

  String  rb   = atSend(cmd, (st == START_AGPS) ? 20000 : 10000, false);
  bool    bok  = rb.indexOf("OK") >= 0;

  if (!bok && st == START_AGPS) {            // el manual V1.02 lo llama CGNSSAGPS
    logf("[GNSS] CAGPS no paso; pruebo AT+CGNSSAGPS");
    rb  = atSend("AT+CGNSSAGPS", 20000, false);
    bok = rb.indexOf("OK") >= 0;
    noteOut = bok ? "CGNSSAGPS OK" : "ni CAGPS ni CGNSSAGPS soportados";
  } else {
    noteOut = String(cmd) + (bok ? " OK" : " NO soportado");
  }

  logf("[GNSS] empujon %s a los %lu ms del encendido: %s",
       cmd, (unsigned long)off, bok ? "aceptado" : "RECHAZADO");
  return t0;
}

static void coolDown(const char *tag) {
  logf("[%s ] GNSS apagado, enfriando %lu s", tag, (unsigned long)GNSS_OFF_WAIT_S);
  uint32_t w0 = millis();
  while (millis() - w0 < GNSS_OFF_WAIT_S * 1000UL) {
    delay(1000);
    uint32_t el = (millis() - w0) / 1000UL;
    if (el % 30 == 0 && el > 0) {
      logf("[%s ] enfriando %lu/%lu s", tag, (unsigned long)el,
           (unsigned long)GNSS_OFF_WAIT_S);
    }
  }
}

// =============================================================================
//  Sondeo de capacidades
// =============================================================================

// Lecturas informativas con el motor ENCENDIDO. Ninguna arranca ni detiene
// nada. Si alguna da ERROR puede ser que el comando no exista o que no admita
// esa forma: es una pista, no una sentencia.
static void probeGnssCapabilities() {
  static const char *probes[] = {
    "AT+CGNSSPROD",     // que motor GNSS lleva dentro
    "AT+CGNSSMODE?",    // constelaciones activas (solo valido con GNSS on)
    "AT+CGNSSRTC?",     // modo RTC del GNSS, 21.2.12 del manual V1.02
    "AT+CGNSSSLEEP=?",  // existe el sueno del motor?
    "AT+CGNSSFLP=?",    // ahorro periodico del motor, 21.2.17
    "AT+CGNSSAGPS=?",   // A-GNSS con el nombre del V1.02
  };
  Serial.println();
  logf("[LAB ] sondeo de capacidades con el motor ENCENDIDO (informativo)");
  for (unsigned i = 0; i < sizeof(probes) / sizeof(probes[0]); i++) {
    atSend(probes[i], 4000, false);
  }
}

// CGPSHOT no tiene forma de test: la unica manera de saber si existe es
// ejecutarlo. Se hace al final del baseline, con el fix ya conseguido y justo
// antes de apagar el motor, para no perturbar ninguna medida. Un hot start es
// no destructivo por definicion: reusa lo que ya hay.
// CGPSWARM y CGPSCOLD NO se sondean aqui a proposito: descartan efemerides y
// dejarian frias las corridas siguientes. CGPSWARM se mide en T5, de ultimo.
static void probeHotCommand() {
  logf("[LAB ] pruebo AT+CGPSHOT ejecutandolo (no tiene forma de test)");
  String r  = atSend("AT+CGPSHOT", 10000, false);
  bool   ok = r.indexOf("OK") >= 0;
  cmdSupport = String("CGPSHOT=") + (ok ? "si" : "no");
  logf("[LAB ] %s%s", cmdSupport.c_str(),
       ok ? " -> la familia CGPSxxx existe en este firmware"
          : " -> ojo: ni ejecutandolo pasa");
}

// =============================================================================
//  Baseline de cielo: exige un fix crudo antes de seguir.
// =============================================================================
static bool warmBaseline(const char *tag) {
  Serial.println();
  Serial.println("-----------------------------------------------------------------");
  logf("[%s ] BASELINE DE CIELO: enciendo GNSS y espero fix crudo (max %lu s)",
       tag, (unsigned long)PREFLIGHT_TIMEOUT_S);
  Serial.println("-----------------------------------------------------------------");

  bool   ready = false;
  String note  = "-";
  uint32_t t0  = gnssStart(START_PWR, ready, note);

  uint32_t nextPoll = millis();
  int   idx = 0, bestSats = 0;
  float lastHdop = -1.0f;
  bool  got = false;
  uint32_t ttff = 0;
  String   stamp = "-";

  while (millis() - t0 < PREFLIGHT_TIMEOUT_S * 1000UL) {
    if (millis() < nextPoll) { delay(20); continue; }
    nextPoll = millis() + POLL_PERIOD_MS;
    idx++;
    GnssSample s = gnssPoll();
    if (s.sats > bestSats) bestSats = s.sats;
    if (s.hdop > 0) lastHdop = s.hdop;

    if (isStale(s)) {
      staleTotal++;
      logf("[%s ] TRAMA RANCIA: misma marca del fix anterior (%s). Sigo sondeando.",
           tag, s.stamp.c_str());
      continue;
    }
    if (s.hasFix) {
      got   = true;
      ttff  = millis() - t0;
      stamp = s.stamp.length() ? s.stamp : String("-");
      notePriorFix(s);
      break;
    }
    if (idx % POLL_LOG_EVERY == 0) {
      logf("[%s ] t=%lus sats=%d hdop=%.1f (sin fix aun)", tag,
           (unsigned long)((millis() - t0) / 1000), s.sats, s.hdop);
    }
  }

  if (got) {
    skyOk     = true;
    skyTtffMs = ttff;
    skySats   = bestSats;
    skyHdop   = lastHdop;
    skyStamp  = stamp;
    logf("[%s ] fix crudo a los %lu s  sats=%d hdop=%.1f marca=%s -> el cielo alcanza",
         tag, (unsigned long)(ttff / 1000), bestSats, lastHdop, stamp.c_str());

    if (stamp == "-" || stamp.length() < 2) {
      stampAvail = false;
      logf("[%s ] la trama NO trae fecha/hora: el guardia de trama rancia queda INACTIVO",
           tag);
    }

    // El motor esta encendido y con fix: el unico momento bueno para preguntar.
    if (cmdSupport == "(no probado)") {
      probeGnssCapabilities();
      probeHotCommand();
    }
    gnssOff();
    coolDown(tag);
    return true;
  }

  gnssOff();
  logf("[%s ] SIN fix crudo en %lu s. Mejor conteo de satelites: %d",
       tag, (unsigned long)PREFLIGHT_TIMEOUT_S, bestSats);
  return false;
}

// =============================================================================
//  Ejecucion de una corrida
// =============================================================================
static void runOne(const RunCfg &cfg, RunResult &out, bool coolAfter) {
  Serial.println();
  Serial.println("-----------------------------------------------------------------");
  logf("[%s ] INICIO  %s   pre=%s  start=%s",
       cfg.id, cfg.label, preName(cfg.pre), startName(cfg.start));
  Serial.println("-----------------------------------------------------------------");

  out.ran  = true;
  out.batV = readBatV();
  logf("[%s ] Vbat aprox %.2f V", cfg.id, out.batV);
  if (lastFixStamp.length() >= 2) {
    logf("[%s ] marca del fix anterior: %s (cualquier trama con esta marca se descarta)",
         cfg.id, lastFixStamp.c_str());
  }

  // ---- variable bajo prueba -------------------------------------------------
  switch (cfg.pre) {
    case PRE_LONG_OFF:        out.note = longOffWait();                    break;
    case PRE_DTR_SLEEP:       out.note = dtrSleepCycle(DTR_SLEEP_S);       break;
    case PRE_DTR_SLEEP_SHORT: out.note = dtrSleepCycle(DTR_SLEEP_SHORT_S); break;
    case PRE_CSCLK_AWAKE:     out.note = csclkAwake();                     break;
    case PRE_GNSS_SLEEP:      out.note = gnssSleepCycle();                 break;
    case PRE_CPOF_PWRKEY:     out.note = modemColdCycle();                 break;
    case PRE_CRESET:          out.note = modemReset();                     break;
    case PRE_PWRKEY_PULSE:    out.note = pwrkeyOnLiveModem();              break;
    default:                  out.note = "-";                              break;
  }

  // Cuanto lleva el GNSS sin trabajar justo antes de arrancarlo. Es la variable
  // que confundia la lectura de R3 en la v1.1; desde la v1.2 va en el CSV.
  out.offTotalS = gnssOffSinceMs ? (millis() - gnssOffSinceMs) / 1000UL : 0;
  logf("[%s ] el GNSS lleva %lu s sin trabajar", cfg.id, (unsigned long)out.offTotalS);

  // ---- arranque -------------------------------------------------------------
  bool   ready     = false;
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

    // ---- guardia de trama rancia -------------------------------------------
    // Fix con la MISMA fecha/hora del fix anterior. No es una medida nueva: es
    // la ultima solucion guardada. Pasa el gate con numeros impecables, asi que
    // el gate no la puede atrapar. Solo la marca de tiempo la delata.
    bool stale = isStale(s);
    if (stale) {
      out.pollsStale++;
      staleTotal++;
      if (out.pollsStale == 1) {
        logf("[%s ] t=%lus TRAMA RANCIA: fix con la marca del fix anterior (%s), "
             "sats=%d hdop=%.1f. No es nueva. Sigo cronometrando.",
             cfg.id, (unsigned long)((millis() - t0) / 1000),
             s.stamp.c_str(), s.sats, s.hdop);
      }
    }
    bool fresh = s.hasFix && !stale;

    // Trama envenenada: lat/lon presentes pero sin satelites ni HDOP. Es la
    // ultima posicion conocida, no una medida. El gate de main.cpp la mata.
    bool poisoned = fresh && (s.sats == 0 || s.hdop <= 0.0f);
    if (poisoned) {
      logf("[%s ] t=%lus TRAMA FANTASMA: fix con sats=%d hdop=%.1f (posicion vieja)",
           cfg.id, (unsigned long)((millis() - t0) / 1000), s.sats, s.hdop);
    }

    bool passesGate = fresh &&
                      s.sats >= MIN_VALID_SATELLITES &&
                      s.hdop > 0.0f && s.hdop <= MAX_VALID_HDOP;

    if (fresh && !out.fixRaw) {
      out.fixRaw    = true;
      out.ttffRawMs = millis() - t0;
      graceDeadline = millis() + POSTFIX_GRACE_S * 1000UL;
      logf("[%s ] *** FIX CRUDO a los %lu s   sats=%d hdop=%.1f marca=%s",
           cfg.id, (unsigned long)(out.ttffRawMs / 1000), s.sats, s.hdop,
           s.stamp.length() ? s.stamp.c_str() : "-");
    }
    if (fresh) notePriorFix(s);

    if (passesGate && !out.fixValid) {
      out.fixValid  = true;
      out.ttffValMs = millis() - t0;
      out.sats      = s.sats;
      out.hdop      = s.hdop;
      out.stamp     = s.stamp.length() ? s.stamp : String("-");
      logf("[%s ] *** FIX QUE PASA EL GATE a los %lu s   sats=%d hdop=%.1f",
           cfg.id, (unsigned long)(out.ttffValMs / 1000), s.sats, s.hdop);
      break;
    }

    bool interesting = (s.sats != lastSats) || (pollIdx % POLL_LOG_EVERY == 0);
    if (interesting && !stale) {
      logf("[%s ] t=%lus poll=%d fix=%s sats=%d hdop=%.1f vacios=%u rancias=%u err=%u",
           cfg.id, (unsigned long)((millis() - t0) / 1000), pollIdx,
           s.hasFix ? "si" : "no", s.sats, s.hdop,
           (unsigned)out.pollsEmpty, (unsigned)out.pollsStale,
           (unsigned)out.pollsError);
      lastSats = s.sats;
    }

    if (graceDeadline && !out.fixValid && millis() > graceDeadline) {
      logf("[%s ] fix crudo sin pasar el gate en %lu s de gracia: corto aqui",
           cfg.id, (unsigned long)POSTFIX_GRACE_S);
      if (out.note == "-") out.note = "fix crudo sin pasar el gate";
      else                 out.note += "; fix crudo sin pasar el gate";
      break;
    }
  }

  if (!out.fixRaw) {
    logf("[%s ] sin fix crudo en %lu s (techo de la prueba)",
         cfg.id, (unsigned long)FIX_TIMEOUT_S);
    if (out.note == "-") out.note = "sin fix crudo";
    else                 out.note += "; sin fix crudo";
  }
  if (out.pollsStale) {
    String extra = String("descarto ") + String((unsigned)out.pollsStale) +
                   " trama(s) rancia(s)";
    if (out.note == "-") out.note = extra;
    else                 out.note += "; " + extra;
  }

  out.modeAfter = fieldAfter(atSend("AT+CGNSSMODE?", 3000, false), "+CGNSSMODE:");
  logf("[%s ] CGNSSMODE con GNSS encendido: %s", cfg.id, out.modeAfter.c_str());

  gnssOff();
  logf("[%s ] FIN", cfg.id);
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
  Serial.println(" RESUMEN GNSS LAB 1.4 - arranque asistido, A-GNSS y sueno del motor");
  Serial.println("=================================================================");
  if (skyOk) {
    Serial.printf(" cielo (baseline): fix crudo en %lu s, sats %d, hdop %.1f\r\n",
                  (unsigned long)(skyTtffMs / 1000), skySats, skyHdop);
  } else {
    Serial.println(" cielo (baseline): SIN FIX. El sitio no da para esta prueba.");
  }
  Serial.printf(" red: %s (IP %s)\r\n", netOk ? "arriba" : "ABAJO", netIp.c_str());
  Serial.printf(" familia CGPSxxx: %s\r\n", cmdSupport.c_str());
  Serial.printf(" guardia de trama rancia: %s, %u descartada(s) en total\r\n",
                stampAvail ? "activo" : "INACTIVO (la trama no trae fecha/hora)",
                (unsigned)staleTotal);
  if (rechargesUsed)        Serial.printf(" recargas de NVRAM usadas: %d\r\n", rechargesUsed);
  if (abortReason.length()) Serial.printf(" ABORTADO: %s\r\n", abortReason.c_str());
  Serial.println("-----------------------------------------------------------------");

  Serial.printf("%-4s %-13s %-13s %-7s %7s %8s %8s %5s %5s %6s %7s %5s  %s\r\n",
                "run", "variable", "pre", "start", "off_s", "ttff_raw", "ttff_val",
                "sats", "hdop", "vacios", "rancias", "err", "nota");

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
    if (r.hdop > 0) snprintf(hdop, sizeof(hdop), "%.1f", r.hdop);
    else            snprintf(hdop, sizeof(hdop), "-");

    Serial.printf("%-4s %-13s %-13s %-7s %6lus %8s %8s %5s %5s %6u %7u %5u  %s\r\n",
                  RUNS[i].id, RUNS[i].label, preName(RUNS[i].pre), startName(RUNS[i].start),
                  (unsigned long)r.offTotalS, raw, val, sats, hdop,
                  (unsigned)r.pollsEmpty, (unsigned)r.pollsStale,
                  (unsigned)r.pollsError, r.note.c_str());
  }

  Serial.println();
  Serial.println("=== CSV (pegar en el log de sesion) ===");
  Serial.println("run,variable,pre,start,off_s,ttff_raw_s,ttff_valid_s,sats,hdop,"
                 "polls_vacios,polls_rancios,polls_error,polls_total,ready,mode_post,"
                 "vbat,utc_fix,nota");
  for (int i = 0; i < RUN_COUNT; i++) {
    if (!results[i].ran) continue;
    RunResult &r = results[i];
    String note = r.note;
    note.replace(",", ";");
    Serial.printf("%s,%s,%s,%s,%lu,%ld,%ld,%d,%.1f,%u,%u,%u,%u,%s,%s,%.2f,%s,%s\r\n",
                  RUNS[i].id, RUNS[i].label, preName(RUNS[i].pre), startName(RUNS[i].start),
                  (unsigned long)r.offTotalS,
                  r.fixRaw   ? (long)(r.ttffRawMs / 1000) : -1L,
                  r.fixValid ? (long)(r.ttffValMs / 1000) : -1L,
                  r.fixValid ? r.sats : -1,
                  r.hdop,
                  (unsigned)r.pollsEmpty, (unsigned)r.pollsStale,
                  (unsigned)r.pollsError, (unsigned)r.pollsTotal,
                  r.ready ? "si" : "no", r.modeAfter.c_str(), r.batV,
                  r.stamp.c_str(), note.c_str());
  }
  Serial.printf("P0,baseline_cielo,-,pwr,0,%ld,-1,%d,%.1f,0,0,0,0,no,-,0.00,%s,%s\r\n",
                skyOk ? (long)(skyTtffMs / 1000) : -1L, skySats, skyHdop,
                skyStamp.c_str(),
                skyOk ? "cielo suficiente" : "sin fix en el baseline");
  Serial.println("=== fin CSV ===");
  Serial.println();

  Serial.println("Como se lee, en orden de importancia:");
  Serial.println(" 0. La columna 'rancias' primero. Si sale mayor que cero en cualquier");
  Serial.println("    corrida, el modem SI devuelve la posicion vieja al encender y eso es");
  Serial.println("    un defecto de produccion: main.cpp publicaria esa posicion como");
  Serial.println("    actual. Hay que meter el chequeo de fecha/hora en readGpsPoint (F22).");
  Serial.println("    Si sale cero en todas, el fix de 0 s de T4 en la v1.2 fue una");
  Serial.println("    readquisicion genuina y tambien es buena noticia.");
  Serial.println(" 1. T5 (warm) DEBE salir mas lento que T1. Warm start descarta efemerides");
  Serial.println("    por definicion. Si T5 sale igual que T1, el firmware esta IGNORANDO la");
  Serial.println("    familia CGPSxxx y entonces T4 no prueba nada: todo el bloque se cae.");
  Serial.println(" 2. Con T5 mas lento, T4 (hot) ya se puede leer:");
  Serial.println("      T4 < T1  -> hay que meter CGPSHOT en pmGnssOn. Entra en F19.");
  Serial.println("      T4 = T1  -> CGNSSPWR=1 ya hace hot start solo. Tema cerrado.");
  Serial.println(" 3. T9 (agps) decide el escenario de 24 h estacionado, donde el frio es");
  Serial.println("    inevitable porque el corte de energia borra las efemerides.");
  Serial.println("    Comparar T9 contra los 117 s de R0, NO contra T1.");
  Serial.println("      T9 muy por debajo de 117 s -> A-GNSS entra al tracker y cambia F23.");
  Serial.println("      Si la IP salio ERROR, T9 no es valida: no concluir nada de ella.");
  Serial.println(" 4. T10 (gnss_sleep) decide el estacionado corto:");
  Serial.println("      T10 << T1 y rancias=0 -> dormir el motor en vez de apagarlo. Falta");
  Serial.println("                   medir consumo con el multimetro antes de cambiar nada.");
  Serial.println("      T10 << T1 pero rancias>0 -> era la trama vieja. No cambiar nada.");
  Serial.println("      nota 'CGNSSSLEEP no soportado' -> tema cerrado, seguimos apagando.");
  Serial.println(" 5. off_s tiene que salir parecido en T1, T4, T5 y T9 (~150 s). Si alguna");
  Serial.println("    se dispara, esa corrida no es comparable y hay que repetirla.");
  Serial.println(" 6. Piso de ruido medido el 2026-08-22 con cielo bueno: +/- 4 s (T1=22 s");
  Serial.println("    y T5=26 s en la misma condicion). Diferencias menores no significan");
  Serial.println("    nada.");
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
  Serial.println(" sondeo   : cada 1 s desde el instante del encendido");
  Serial.println(" gate     : sats >= 5 y HDOP <= 2.5 (igual que main.cpp)");
  Serial.println(" guardia  : se descarta toda trama con la marca de tiempo del fix previo");
  Serial.printf(" filtro   : P0 exige fix crudo en %lu s o aborta\r\n",
                (unsigned long)PREFLIGHT_TIMEOUT_S);
  Serial.println(" duracion : ~20-25 min tipico, hasta ~40 min en el peor caso");
  Serial.println("=================================================================");
  Serial.println("  red: CEREG + NETOPEN + IPADDR una vez, antes del baseline");
  Serial.println("  P0  baseline_cielo  filtro de sitio, efemerides frescas y capacidades");
  for (int i = 0; i < RUN_COUNT; i++) {
    Serial.printf("  %-4s %-15s pre=%-14s start=%-7s %s\r\n",
                  RUNS[i].id, RUNS[i].label, preName(RUNS[i].pre),
                  startName(RUNS[i].start), RUNS[i].enabled ? "" : "(desactivada)");
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

  // La familia CGPSxxx NO se sondea aqui: solo es valida con el motor GNSS
  // encendido y no admite la forma =?. Ese fue el falso negativo de la v1.2.
  // Se prueba dentro del baseline, ejecutandola de verdad.

  netOk = netUp();

  logf("[LAB ] dejo el GNSS apagado 20 s para partir de un estado conocido");
  gnssOff();
  delay(20000);

  // ---- P0: filtro de sitio, efemerides frescas y sondeo de capacidades -----
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
