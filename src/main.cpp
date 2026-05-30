#include <Arduino.h>

#define TINY_GSM_RX_BUFFER 1024
#define SerialMon Serial
#define TINY_GSM_DEBUG SerialMon

// Modem (ajusta si tu fork pide otra macro)
#define TINY_GSM_MODEM_SIM7670G
#include <TinyGsmClient.h>
#include <PubSubClient.h>

// ---------- Pines ----------
#define MODEM_BAUDRATE 115200
#define MODEM_TX_PIN        4
#define MODEM_RX_PIN        5
#define MODEM_DTR_PIN       7
#define BOARD_PWRKEY_PIN    46

#define MODEM_GPS_ENABLE_GPIO   1
#define MODEM_GPS_ENABLE_LEVEL  1

#define MODEM_POWERON_PULSE_WIDTH_MS 1000

HardwareSerial SerialAT(1);
TinyGsm modem(SerialAT);

// ---------- LTE / MQTT config ----------
const char APN[]  = "internet";      // <-- PON AQUÍ el APN real de tu SIM
const char USER[] = "";              // normalmente vacío
const char PASS[] = "";              // normalmente vacío

const char MQTT_HOST[] = "mqtt.julidcardenas.site";
const int  MQTT_PORT   = 8883;       // TLS
const char MQTT_USER[] = "julian";
const char MQTT_PASS[] = "8aDpW3sm9BLZKS";  // <-- tu pass real

const char DEVICE_ID[] = "Lilygo";
const char TOPIC_TELEMETRY[] = "tracker/Lilygo/telemetria";

// TLS client para el modem
TinyGsmClient netClient(modem);
PubSubClient mqtt(netClient);

// ---------- Timing ----------
static const uint32_t GPS_PERIOD_MS = 10000;

// ---------- Helpers ----------
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

static void waitForAT() {
  SerialMon.println("Start modem...");
  delay(3000);
  while (!modem.testAT(1000)) {
    SerialMon.println("AT fail, retrying...");
    delay(1000);
  }
  SerialMon.println("AT OK");
}

static void ensureLTE() {
  // En algunos ejemplos conviene: modem.restart(); pero tú ya tienes power sequence estable.
  SerialMon.print("Waiting for network...");
  if (!modem.waitForNetwork(60000L)) {
    SerialMon.println(" FAIL");
    return;
  }
  SerialMon.println(" OK");

  SerialMon.print("Connecting GPRS/APN...");
  if (!modem.gprsConnect(APN, USER, PASS)) {
    SerialMon.println(" FAIL");
    return;
  }
  SerialMon.println(" OK");

  IPAddress ip = modem.localIP();
  SerialMon.print("IP: ");
  SerialMon.println(ip);
}

static void ensureMQTT() {
  mqtt.setServer(MQTT_HOST, MQTT_PORT);

  // Para pruebas rápidas TLS sin validar certificado:
  // (equivalente a --insecure en mosquitto_sub)
  // tlsClient.setInsecure();

  while (!mqtt.connected()) {
    SerialMon.print("[MQTT] Connecting...");
    String clientId = String("logan-gps-") + String((uint32_t)ESP.getEfuseMac(), HEX);

    bool ok = mqtt.connect(clientId.c_str(), MQTT_USER, MQTT_PASS);
    SerialMon.println(ok ? " OK" : " FAIL");

    if (!ok) delay(2000);
  }
}

void setup() {
  SerialMon.begin(115200);
  delay(200);

  modemPowerOn();
  SerialAT.begin(MODEM_BAUDRATE, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);

  waitForAT();

  // 1) LTE up
  ensureLTE();

  // 2) MQTT up
  ensureMQTT();

  // 3) GPS on
  SerialMon.println("Enabling GPS...");
  while (!modem.enableGPS(MODEM_GPS_ENABLE_GPIO, MODEM_GPS_ENABLE_LEVEL)) {
    SerialMon.print(".");
    delay(500);
  }
  SerialMon.println("\nGPS Enabled");
  modem.setGPSBaud(115200);
}

void loop() {
  // Mantener sesión MQTT viva
  if (!modem.isNetworkConnected() || !modem.isGprsConnected()) {
    SerialMon.println("[NET] down -> reconnect");
    ensureLTE();
  }
  if (!mqtt.connected()) {
    SerialMon.println("[MQTT] down -> reconnect");
    ensureMQTT();
  }
  mqtt.loop();

  static uint32_t last = 0;
  if (millis() - last >= GPS_PERIOD_MS) {
    last = millis();

    float lat=0, lon=0, speed=0, alt=0, acc=0;
    int vsat=0, usat=0;
    int year=0, month=0, day=0, hour=0, min=0, sec=0;
    uint8_t fix=0;

    bool ok = modem.getGPS(&fix, &lat, &lon, &speed, &alt, &vsat, &usat, &acc,
                           &year, &month, &day, &hour, &min, &sec);

    if (!ok) {
      SerialMon.println("[GPS] read FAIL");
      return;
    }

    // JSON compacto (sin ArduinoJson por ahora)
    char payload[256];
    snprintf(payload, sizeof(payload),
      "{\"device_id\":\"%s\",\"fix\":%u,\"lat\":%.6f,\"lon\":%.6f,"
      "\"speed\":%.2f,\"alt\":%.1f,\"vsat\":%d,\"acc\":%.2f,"
      "\"ts\":\"%04d-%02d-%02dT%02d:%02d:%02dZ\"}",
      DEVICE_ID, fix, lat, lon, speed, alt, vsat, acc,
      year, month, day, hour, min, sec
    );

    bool pubOk = mqtt.publish(TOPIC_TELEMETRY, payload);
    SerialMon.printf("[PUB] %s topic=%s payload=%s\n", pubOk ? "OK" : "FAIL", TOPIC_TELEMETRY, payload);
  }

  delay(10);
}