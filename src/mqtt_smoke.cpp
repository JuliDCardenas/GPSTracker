#define TINY_GSM_RX_BUFFER 1024
#define SerialMon Serial
#define TINY_GSM_DEBUG SerialMon

#define TINY_GSM_MODEM_SIM7670G

#include <Arduino.h>
#include <TinyGsmClient.h>
#include <PubSubClient.h>

// =====================================================
// GPS Tracker Logan - MQTT Smoke Test
// Board: LilyGO T-SIM7670G-S3-Standard
// Objetivo: probar firmware -> LTE -> MQTT publish
// Sin ACC, sin GPS, sin sleep.
// =====================================================

// ---------- Pines modem ----------
#define MODEM_BAUDRATE 115200
#define MODEM_TX_PIN 4
#define MODEM_RX_PIN 5
#define MODEM_DTR_PIN 7
#define BOARD_PWRKEY_PIN 46

#define MODEM_POWERON_PULSE_WIDTH_MS 1000

// ---------- Objetos ----------
HardwareSerial SerialAT(1);
TinyGsm modem(SerialAT);
TinyGsmClient netClient(modem);
PubSubClient mqtt(netClient);

// ---------- LTE ----------
const char APN[] = "internet";
const char APN_USER[] = "";
const char APN_PASS[] = "";

// ---------- MQTT ----------
// OJO: por ahora usa las credenciales que ya tienes en tu main.cpp.
// No las pegues en el chat.
const char MQTT_HOST[] = "mqtt.julidcardenas.site";
const int MQTT_PORT = 1883;
const char MQTT_USER[] = "julian";
const char MQTT_PASS[] = "8aDpW3sm9BLZKS";

const char DEVICE_ID[] = "Lilygo";
const char TOPIC_TEST[] = "tracker/Lilygo/test";

// ---------- Timing ----------
static const uint32_t PUBLISH_EVERY_MS = 10000;
static uint32_t lastPublishMs = 0;
static uint32_t seq = 0;

// =====================================================
// Modem helpers
// =====================================================

static void modemPowerOn() {
  SerialMon.println("[MODEM] Power key sequence...");

  pinMode(MODEM_DTR_PIN, OUTPUT);
  digitalWrite(MODEM_DTR_PIN, LOW);

  pinMode(BOARD_PWRKEY_PIN, OUTPUT);

  // Misma secuencia que veníamos usando en main.cpp
  digitalWrite(BOARD_PWRKEY_PIN, LOW);
  delay(100);
  digitalWrite(BOARD_PWRKEY_PIN, HIGH);
  delay(MODEM_POWERON_PULSE_WIDTH_MS);
  digitalWrite(BOARD_PWRKEY_PIN, LOW);

  delay(3000);
}

static bool waitForAT(uint32_t timeoutMs = 20000) {
  SerialMon.println("[MODEM] Waiting for AT...");

  uint32_t t0 = millis();
  while (!modem.testAT(1000)) {
    SerialMon.println("[MODEM] AT fail, retrying...");
    if (millis() - t0 > timeoutMs) {
      SerialMon.println("[MODEM] AT timeout");
      return false;
    }
    delay(500);
  }

  SerialMon.println("[MODEM] AT OK");
  return true;
}

static bool connectLTE() {
  SerialMon.println("[LTE] Checking SIM...");
  String simStatus = modem.getSimStatus() == 3 ? "READY" : "NOT_READY";
  SerialMon.print("[LTE] SIM status: ");
  SerialMon.println(simStatus);

  SerialMon.println("[LTE] Waiting for network...");
  if (!modem.waitForNetwork(60000L)) {
    SerialMon.println("[LTE] Network FAIL");
    return false;
  }
  SerialMon.println("[LTE] Network OK");

  SerialMon.print("[LTE] Operator: ");
  SerialMon.println(modem.getOperator());

  SerialMon.print("[LTE] Signal quality CSQ: ");
  SerialMon.println(modem.getSignalQuality());

  SerialMon.println("[LTE] Connecting APN...");
  if (!modem.gprsConnect(APN, APN_USER, APN_PASS)) {
    SerialMon.println("[LTE] APN/GPRS FAIL");
    return false;
  }

  SerialMon.println("[LTE] APN/GPRS OK");

  SerialMon.print("[LTE] Local IP: ");
  SerialMon.println(modem.localIP());

  return true;
}

static bool connectMQTT() {
  mqtt.setServer(MQTT_HOST, MQTT_PORT);

  SerialMon.print("[MQTT] Connecting to ");
  SerialMon.print(MQTT_HOST);
  SerialMon.print(":");
  SerialMon.println(MQTT_PORT);

  uint32_t t0 = millis();

  while (!mqtt.connected()) {
    String clientId = String("logan-smoke-") + String((uint32_t)ESP.getEfuseMac(), HEX);

    SerialMon.print("[MQTT] Client ID: ");
    SerialMon.println(clientId);

    bool ok = mqtt.connect(clientId.c_str(), MQTT_USER, MQTT_PASS);

    if (ok) {
      SerialMon.println("[MQTT] Connected OK");
      return true;
    }

    SerialMon.print("[MQTT] Connect FAIL, state=");
    SerialMon.println(mqtt.state());

    if (millis() - t0 > 30000) {
      SerialMon.println("[MQTT] Connect timeout");
      return false;
    }

    delay(3000);
  }

  return true;
}

static bool ensureConnections() {
  if (!modem.isNetworkConnected() || !modem.isGprsConnected()) {
    SerialMon.println("[LTE] Connection down, reconnecting...");
    if (!connectLTE()) return false;
  }

  if (!mqtt.connected()) {
    SerialMon.println("[MQTT] Disconnected, reconnecting...");
    if (!connectMQTT()) return false;
  }

  return true;
}

static void publishSmoke() {
  char payload[192];

  snprintf(
    payload,
    sizeof(payload),
    "{\"device\":\"%s\",\"test\":\"mqtt_smoke\",\"seq\":%lu,\"ms\":%lu}",
    DEVICE_ID,
    (unsigned long)seq++,
    (unsigned long)millis()
  );

  bool ok = mqtt.publish(TOPIC_TEST, payload);

  SerialMon.print("[PUB] ");
  SerialMon.print(ok ? "OK" : "FAIL");
  SerialMon.print(" topic=");
  SerialMon.print(TOPIC_TEST);
  SerialMon.print(" payload=");
  SerialMon.println(payload);
}

// =====================================================
// Arduino
// =====================================================

void setup() {
  SerialMon.begin(115200);
  delay(2000);

  SerialMon.println();
  SerialMon.println("=================================");
  SerialMon.println("GPS Tracker Logan - MQTT Smoke");
  SerialMon.println("=================================");
  SerialMon.println("[BOOT] Starting minimal LTE/MQTT test");
  SerialMon.println("[BOOT] No ACC, no GPS, no sleep");

  modemPowerOn();

  SerialAT.begin(MODEM_BAUDRATE, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);

  if (!waitForAT()) {
    SerialMon.println("[FATAL] Modem not responding");
    return;
  }

  modem.restart();

  if (!connectLTE()) {
    SerialMon.println("[FATAL] LTE failed");
    return;
  }

  if (!connectMQTT()) {
    SerialMon.println("[FATAL] MQTT failed");
    return;
  }

  publishSmoke();
  lastPublishMs = millis();
}

void loop() {
  if (!ensureConnections()) {
    SerialMon.println("[LOOP] Connection not ready, retrying soon...");
    delay(5000);
    return;
  }

  mqtt.loop();

  if (millis() - lastPublishMs >= PUBLISH_EVERY_MS) {
    lastPublishMs = millis();
    publishSmoke();
  }

  delay(10);
}