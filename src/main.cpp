#include "secrets.h"

#define TINY_GSM_RX_BUFFER 1024
#define SerialMon Serial
#define TINY_GSM_DEBUG SerialMon

// Modem
#define TINY_GSM_MODEM_SIM7670G
#include <TinyGsmClient.h>
#include <PubSubClient.h>

// =====================================================
// GPS Tracker Logan - ACC sense + LTE/MQTT/GPS + aviso "parked"
// Board: LilyGO T-SIM7670G-S3-Standard (ESP32-S3)
// Framework: Arduino (PlatformIO)
// =====================================================

// ---------- Pines modem ----------
#define MODEM_BAUDRATE          115200
#define MODEM_TX_PIN            4
#define MODEM_RX_PIN            5
#define MODEM_DTR_PIN           7
#define BOARD_PWRKEY_PIN        46

#define MODEM_GPS_ENABLE_GPIO   1   // GPIO interno del MODEM (no el del ESP32)
#define MODEM_GPS_ENABLE_LEVEL  1

#define MODEM_POWERON_PULSE_WIDTH_MS 1000

// ---------- ACC sense (ESP32) ----------
// OJO: ADC1_CH0 = GPIO1 del ESP32-S3. Es independiente del "MODEM_GPS_ENABLE_GPIO 1".
static const int   ACC_ADC_GPIO = 1;      // ADC1_CH0 (GPIO1 del ESP32)
static const float ACC_V_ON     = 2.00f;  // histéresis ON
static const float ACC_V_OFF    = 1.20f;  // histéresis OFF

static const uint32_t SAMPLE_MS          = 50;
static const uint32_t ACC_ON_CONFIRM_MS  = 300;
static const uint32_t ACC_OFF_CONFIRM_MS = 5000;   // > crank (~2s)
static const uint32_t MODEM_OFF_GRACE_MS = 30000;  // ventana para buscar fix fresco antes de apagar

// ---------- Objetos modem / MQTT ----------
HardwareSerial SerialAT(1);
TinyGsm        modem(SerialAT);
TinyGsmClient  netClient(modem);
PubSubClient   mqtt(netClient);

// ---------- LTE / MQTT config ----------
const char APN[]  = "internet"; // <-- APN real de tu SIM
const char USER[] = "";
const char PASS[] = "";

const char MQTT_HOST[] = "mqtt.julidcardenas.site";
const int  MQTT_PORT   = 1883;  // sin TLS
const char MQTT_USER[] = "julian";
const char MQTT_PASS[] = "8aDpW3sm9BLZKS";

const char DEVICE_ID[]       = "Lilygo";
const char TOPIC_TELEMETRY[] = "tracker/Lilygo/telemetria";

static const uint32_t GPS_PERIOD_MS = 5000; // latencia de envío

// ---------- State machine ----------
enum class PowerState : uint8_t { NORMAL = 0, MODEM_OFF = 1 };
static PowerState g_state = PowerState::NORMAL;

static bool     g_accLogical       = false;
static bool     g_accCandidate     = false;
static uint32_t g_candidateStartMs = 0;
static uint32_t g_offGraceStartMs  = 0;
static bool     g_modemReady       = false; // modem+LTE+GPS listos
static bool     g_modemSleeping    = false; // módem dormido (CSCLK+DTR), GNSS off, SIN poweroff
static bool     g_parkedSent       = false; // ya mandamos el aviso 'parked' en este ciclo OFF

// ---------- Cache del último fix bueno ----------
struct GpsFix {
  bool    valid = false;
  uint8_t fix   = 0;
  float   lat = 0, lon = 0, speed = 0, alt = 0, accuracy = 0;
  int     vsat = 0;
  int     year = 0, month = 0, day = 0, hour = 0, minute = 0, sec = 0;
};
static GpsFix g_lastFix;

// =====================================================
// ACC sense
// =====================================================
static float readAccVolts() {
  uint32_t mv = analogReadMilliVolts(ACC_ADC_GPIO);
  return mv / 1000.0f;
}

static void updateAccDecision(float vAcc) {
  bool inst;
  if (g_accLogical) inst = !(vAcc < ACC_V_OFF);
  else              inst = (vAcc > ACC_V_ON);

  if (inst == g_accLogical) {
    g_candidateStartMs = 0;
    g_accCandidate = g_accLogical;
    return;
  }
  if (g_candidateStartMs == 0 || inst != g_accCandidate) {
    g_accCandidate = inst;
    g_candidateStartMs = millis();
    return;
  }
  uint32_t elapsed = millis() - g_candidateStartMs;
  uint32_t need = g_accCandidate ? ACC_ON_CONFIRM_MS : ACC_OFF_CONFIRM_MS;
  if (elapsed >= need) {
    g_accLogical = g_accCandidate;
    g_candidateStartMs = 0;
    SerialMon.printf("[ACC] logical=%d v=%.2fV\n", g_accLogical ? 1 : 0, vAcc);
  }
}

// =====================================================
// Modem / LTE / MQTT / GPS
// =====================================================
static void modemPowerOn() {
  pinMode(MODEM_DTR_PIN, OUTPUT);
  digitalWrite(MODEM_DTR_PIN, LOW);
  pinMode(BOARD_PWRKEY_PIN, OUTPUT);
  digitalWrite(BOARD_PWRKEY_PIN, LOW);
  delay(100);
  digitalWrite(BOARD_PWRKEY_PIN, HIGH);
  delay(MODEM_POWERON_PULSE_WIDTH_MS);
  digitalWrite(BOARD_PWRKEY_PIN, LOW);
}

static bool waitForAT() {
  SerialMon.println("Start modem...");
  delay(3000);
  uint32_t t0 = millis();
  while (!modem.testAT(1000)) {
    SerialMon.println("AT fail, retrying...");
    if (millis() - t0 > 20000) return false;
    delay(1000);
  }
  SerialMon.println("AT OK");
  return true;
}

static bool ensureLTE() {
  SerialMon.print("Waiting for network...");
  if (!modem.waitForNetwork(60000L)) { SerialMon.println(" FAIL"); return false; }
  SerialMon.println(" OK");

  SerialMon.print("Connecting GPRS/APN...");
  if (!modem.gprsConnect(APN, USER, PASS)) { SerialMon.println(" FAIL"); return false; }
  SerialMon.println(" OK");

  SerialMon.print("IP: ");
  SerialMon.println(modem.localIP());
  return true;
}

static bool ensureMQTT() {
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  uint32_t t0 = millis();
  while (!mqtt.connected()) {
    SerialMon.print("[MQTT] Connecting... ");
    String clientId = String("logan-") + String((uint32_t)ESP.getEfuseMac(), HEX);
    if (mqtt.connect(clientId.c_str(), MQTT_USER, MQTT_PASS)) {
      SerialMon.println("OK");
      return true;
    }
    SerialMon.print("FAIL state=");
    SerialMon.println(mqtt.state());
    if (millis() - t0 > 30000) return false;
    delay(2000);
  }
  return true;
}

static void gpsOn() {
  SerialMon.println("Enabling GPS...");
  uint32_t t0 = millis();
  while (!modem.enableGPS(MODEM_GPS_ENABLE_GPIO, MODEM_GPS_ENABLE_LEVEL)) {
    SerialMon.print(".");
    if (millis() - t0 > 10000) { SerialMon.println("\n[GPS] enable timeout"); break; }
    delay(500);
  }
  modem.setGPSBaud(115200);
  SerialMon.println("\nGPS Enabled");
}

static void gpsOff() {
  SerialMon.println("[GPS] off");
  modem.disableGPS();
}

// Despierta el módem desde sleep y reactiva GNSS -> warm/hot start (segundos).
static bool modemWakeGnssOn() {
  SerialMon.println("[MODEM] wake + GNSS on (warm start)");
  digitalWrite(MODEM_DTR_PIN, LOW);   // DTR bajo -> saca el módem del sleep
  delay(50);
  modem.sendAT("+CSCLK=0");           // desactiva slow-clock mientras opera
  modem.waitResponse();

  // Revalidar interfaz AT tras el wake
  uint32_t t0 = millis();
  while (!modem.testAT(1000)) {
    if (millis() - t0 > 10000) { SerialMon.println("[WAKE] AT timeout"); return false; }
    delay(200);
  }

  // Revalidar red + datos + MQTT
  if (!modem.isNetworkConnected() && !modem.waitForNetwork(60000L)) return false;
  if (!modem.isGprsConnected() && !ensureLTE()) return false;
  if (!ensureMQTT()) return false;

  gpsOn();                            // AT+CGNSSPWR=1 -> el GNSS retiene efemérides -> warm/hot
  return true;
}

// Enciende todo el stack. Si veníamos de sleep -> warm start (sin power-cycle).
static bool modemBringUp() {
  if (g_modemSleeping) {
    bool ok = modemWakeGnssOn();
    if (ok) g_modemSleeping = false;
    return ok;
  }
  // Arranque en frío real (primer arranque o tras poweroff)
  modemPowerOn();
  SerialAT.begin(MODEM_BAUDRATE, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);
  if (!waitForAT()) return false;
  if (!ensureLTE())  return false;
  if (!ensureMQTT()) return false;
  gpsOn();
  return true;
}

// OPCIÓN 1: en vez de poweroff, apaga SOLO el GNSS y duerme el módem (CSCLK+DTR).
// El módem queda alimentado -> el GNSS conserva efemérides -> próximo fix en warm/hot start.
static void modemSleepGnssOff() {
  SerialMon.println("[MODEM] GNSS off + sleep (warm start, sin PWRKEY)");
  modem.disableGPS();                 // AT+CGNSSPWR=0 (apaga solo el GNSS)
  modem.gprsDisconnect();             // libera datos; el módem sigue registrado/alimentado
  modem.sendAT("+CSCLK=1");           // habilita sleep controlado por DTR
  modem.waitResponse();
  digitalWrite(MODEM_DTR_PIN, HIGH);  // DTR alto -> el módem entra a sleep
  g_modemSleeping = true;
  g_modemReady    = false;
}

// Lee un fix; lo marca válido si fix>=2
static bool readGps(GpsFix &g) {
  int usat = 0;
  bool ok = modem.getGPS(&g.fix, &g.lat, &g.lon, &g.speed, &g.alt, &g.vsat, &usat, &g.accuracy,
                         &g.year, &g.month, &g.day, &g.hour, &g.minute, &g.sec);
  g.valid = ok && g.fix >= 2;
  return g.valid;
}

// CSV v2: v2,device,fix,lat,lon,speed,alt,vsat,acc,ignition,event,ts
static void buildPayload(char *buf, size_t n, const GpsFix &g, int ignition, const char *event) {
  snprintf(buf, n,
           "v2,%s,%u,%.6f,%.6f,%.2f,%.1f,%d,%.2f,%d,%s,%04d-%02d-%02dT%02d:%02d:%02dZ",
           DEVICE_ID, g.fix, g.lat, g.lon, g.speed, g.alt, g.vsat, g.accuracy,
           ignition, (event && event[0]) ? event : "-",
           g.year, g.month, g.day, g.hour, g.minute, g.sec);
}

// Telemetría normal (ignition=1)
static void publishGps() {
  GpsFix g;
  if (!readGps(g)) { SerialMon.println("[GPS] read FAIL/no-fix"); return; }
  g_lastFix = g; // cache del último fix bueno

  char payload[256];
  buildPayload(payload, sizeof(payload), g, 1, "-");
  bool pubOk = mqtt.publish(TOPIC_TELEMETRY, payload);
  SerialMon.printf("[PUB] %s payload=%s\n", pubOk ? "OK" : "FAIL", payload);
}

// Aviso de estacionado (ignition=0, event=parked) con reintentos antes de apagar
static bool publishParked(const GpsFix &g) {
  char payload[256];
  buildPayload(payload, sizeof(payload), g, 0, "parked");
  bool pubOk = false;
  for (int i = 0; i < 3 && !pubOk; i++) {
    mqtt.loop();
    pubOk = mqtt.publish(TOPIC_TELEMETRY, payload);
    if (!pubOk) delay(300);
  }
  SerialMon.printf("[PARKED] %s payload=%s\n", pubOk ? "OK" : "FAIL", payload);
  return pubOk;
}

// =====================================================
// Estados
// =====================================================
static void runNormal() {
  if (!g_modemReady) {
    SerialMon.println("[NORMAL] bring up modem/LTE/MQTT/GPS");
    g_modemReady = modemBringUp();
    if (!g_modemReady) { delay(2000); return; } // reintenta en el próximo loop
  }

  if (!modem.isNetworkConnected() || !modem.isGprsConnected()) {
    SerialMon.println("[NET] down -> reconnect");
    if (!ensureLTE()) return;
  }
  if (!mqtt.connected()) {
    SerialMon.println("[MQTT] down -> reconnect");
    if (!ensureMQTT()) return;
  }
  mqtt.loop();

  static uint32_t last = 0;
  if (millis() - last >= GPS_PERIOD_MS) {
    last = millis();
    publishGps();
  }
}

static void runModemOff() {
  if (!g_modemReady) return; // ya está apagado, ESP en idle

  // Inicio de la ventana de gracia
  if (g_offGraceStartMs == 0) {
    g_offGraceStartMs = millis();
    g_parkedSent = false;
    SerialMon.println("[MODEM_OFF] grace: busco fix fresco para 'parked'");
  }

  // Mantener viva la conexión MQTT mientras buscamos el fix
  if (!mqtt.connected()) ensureMQTT();
  mqtt.loop();

  // 1) Intentar un fix fresco y mandar 'parked'
  if (!g_parkedSent) {
    GpsFix g;
    if (readGps(g)) {
      g_lastFix = g;
      publishParked(g);
      g_parkedSent = true;
      SerialMon.println("[MODEM_OFF] parked con fix fresco -> sleep + GNSS off");
      modemSleepGnssOff();
      g_offGraceStartMs = 0;
      return;
    }
  }

  // 2) Se acabó la ventana: usar último fix cacheado (si hay) y apagar igual
  if (millis() - g_offGraceStartMs >= MODEM_OFF_GRACE_MS) {
    if (!g_parkedSent && g_lastFix.valid) {
      publishParked(g_lastFix);
      SerialMon.println("[MODEM_OFF] parked con último fix cacheado -> sleep + GNSS off");
    } else if (!g_parkedSent) {
      SerialMon.println("[MODEM_OFF] sin fix disponible; duermo sin 'parked'");
    }
    g_parkedSent = true;
    modemSleepGnssOff();
    g_offGraceStartMs = 0;
  }
}

static void setState(PowerState s) {
  if (g_state == s) return;
  g_state = s;
  g_offGraceStartMs = 0;
  g_parkedSent = false;
  SerialMon.println(s == PowerState::NORMAL ? "[STATE] NORMAL" : "[STATE] MODEM_OFF");
}

// =====================================================
// Arduino
// =====================================================
void setup() {
  SerialMon.begin(115200);
  delay(200);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  float v = readAccVolts();
  g_accLogical = (v > ACC_V_ON);
  g_state = g_accLogical ? PowerState::NORMAL : PowerState::MODEM_OFF;
  SerialMon.printf("[BOOT] vAcc=%.2fV accLogical=%d\n", v, g_accLogical ? 1 : 0);
}

void loop() {
  static uint32_t lastSampleMs = 0;
  if (millis() - lastSampleMs >= SAMPLE_MS) {
    lastSampleMs = millis();
    updateAccDecision(readAccVolts());

    if (g_accLogical && g_state != PowerState::NORMAL)            setState(PowerState::NORMAL);
    else if (!g_accLogical && g_state != PowerState::MODEM_OFF)   setState(PowerState::MODEM_OFF);
  }

  if (g_state == PowerState::NORMAL) runNormal();
  else                               runModemOff();

  delay(5);
}