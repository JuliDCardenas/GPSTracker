// PRUEBA EN CASA (WiFi): sense de ignicion (VBUS post-buck) -> GPIO9
//
// Hardware (baquelita, validado en papel 2026-08-15):
//   VBUS --[47k]--+--[510]--> GPIO9 (ADC1_CH8)
//                 |
//               [68k]
//                 |
//   GND ----------+
//   + 100nF en paralelo con R2 y Zener 3.6V (catodo hacia el pin)
//   VBUS = V_pin x 1.9366   (factor CALIBRADO en banco 2026-08-17)
//
// CALIBRACION DEL DIVISOR (2026-08-17):
//   Medicion en banco, buck alimentado desde el carro:
//     salida del buck (multimetro) = 5.25V
//     V_pin (multimetro)           = 2.74V
//     V_pin (lectura del ADC)      = 2.711V
//   Relacion real = 2.74 / 5.25 = 0.5219 contra 0.5913 teorico de 47k/68k
//   => desviacion -11.7%. Causa probable: fuga inversa del Zener 3.6V
//   (equivalente ~210k / ~13uA) en paralelo con R2, sobre un divisor de muy
//   alta impedancia (~40uA de corriente de trabajo).
//   Factor empirico = 5.25 / 2.711 = 1.9366 (absorbe tambien el ~1% de error
//   del ADC frente al multimetro).
//   LIMITACION: al depender de una fuga, el VBUS reportado puede derivar con
//   la temperatura. La logica ON/OFF no se afecta porque los umbrales estan
//   definidos sobre el pin, no sobre VBUS.
//   PENDIENTE (hardware, tarea en Notion): bajar el divisor a 10k/12k y
//   ajustar el pot del MP1584 a 5.05-5.10V para recuperar una relacion
//   predecible; entonces recalibrar este factor.
//
// Variante de adc_sense_field que usa el WiFi del ESP32-S3 en vez del LTE
// del modem. El modem NO se enciende en esta prueba: una variable menos
// mientras se valida la cadena 12V -> buck -> divisor -> ADC.
// GPIO9 es ADC1_CH8: funciona con WiFi activo. (ADC2 quedaria bloqueado
// con WiFi; por eso el sense quedo en ADC1.)
//
// Requiere WIFI_SSID y WIFI_PASS en include/secrets.h.
//
// Topics (mismo broker del tracker):
//   tracker/Lilygo/adc             -> lectura periodica (~2 s, QoS 0)
//   tracker/Lilygo/event/ignition  -> transiciones IGN ON/OFF (retained)
//   tracker/Lilygo/sys/status      -> "adc_wifi_boot" al conectar (retained)
//   tracker/Lilygo/sys/lwt         -> online/offline (retained)

#include <Arduino.h>
#include <esp_task_wdt.h>
#include <WiFi.h>
#include <PubSubClient.h>

// Credenciales en include/secrets.h (no subir al repo)
#include "secrets.h"

#define SerialMon Serial
#define SENSE_PIN 9  // GPIO9 = ADC1_CH8

WiFiClient netClient;
PubSubClient mqtt(netClient);

// ---------- Topics ----------
const char TOPIC_ADC[]      = "tracker/Lilygo/adc";
const char TOPIC_IGNITION[] = "tracker/Lilygo/event/ignition";
const char TOPIC_STATUS[]   = "tracker/Lilygo/sys/status";
const char TOPIC_LWT[]      = "tracker/Lilygo/sys/lwt";

const char LWT_ONLINE[]  = "online";
const char LWT_OFFLINE[] = "offline";

// ---------- Divisor / ADC ----------
// Factor calibrado 2026-08-17 (antes 1.6912f teorico). Ver nota de arriba.
static const float DIVIDER_FACTOR = 1.9366f;  // VBUS = V_pin * factor
static const float PIN_ON_V  = 2.5f;          // umbral ON medido en el pin
static const float PIN_OFF_V = 0.6f;          // umbral OFF medido en el pin

static const uint32_t SAMPLE_PERIOD_MS = 250;
static const uint32_t REPORT_PERIOD_MS = 2000;
static const int SAMPLES = 32;                // promedio contra ruido del ADC

// Antirrebote (decision documentada 2026-08-15):
//   ON  : > 2.5V sostenido 2-3 s
//   OFF : < 0.6V sostenido 20-30 s (absorbe el crank)
// Para ciclos de banco rapidos se puede bajar OFF_DEBOUNCE_MS a 5000
// temporalmente; el valor de produccion es 20000-30000.
static const uint32_t ON_DEBOUNCE_MS  = 3000;
static const uint32_t OFF_DEBOUNCE_MS = 20000;

// ---------- Robustez ----------
static const uint16_t MQTT_KEEPALIVE_SEC = 60;
static const uint16_t MQTT_SOCKET_TIMEOUT_SEC = 15;
static const uint32_t MQTT_RETRY_MS = 3000;
static const uint8_t MQTT_FAILS_BEFORE_ESP_RESTART = 20;
static const uint32_t WDT_TIMEOUT_SEC = 120;

// ---------- Estado ----------
enum IgnState { IGN_UNKNOWN = 0, IGN_ON, IGN_OFF };

static IgnState ignState = IGN_UNKNOWN;
static IgnState candidate = IGN_UNKNOWN;
static uint32_t candidateSinceMs = 0;

static uint8_t mqttFailCount = 0;
static uint32_t mqttNextAttemptMs = 0;
static bool bootStatusPublished = false;

static const char *stateStr(IgnState s) {
  return s == IGN_ON ? "IGN_ON" : (s == IGN_OFF ? "IGN_OFF" : "UNKNOWN");
}

// ---------- Watchdog ----------
static void watchdogSetup() {
#if ESP_IDF_VERSION_MAJOR >= 5
  esp_task_wdt_config_t wdtConfig = {};
  wdtConfig.timeout_ms = WDT_TIMEOUT_SEC * 1000;
  wdtConfig.idle_core_mask = 0;
  wdtConfig.trigger_panic = true;
  if (esp_task_wdt_reconfigure(&wdtConfig) != ESP_OK) {
    esp_task_wdt_init(&wdtConfig);
  }
#else
  esp_task_wdt_init(WDT_TIMEOUT_SEC, true);
#endif
  esp_task_wdt_add(NULL);
  SerialMon.printf("[WDT] activo timeout=%us\n", (unsigned)WDT_TIMEOUT_SEC);
}

static void watchdogFeed() {
  esp_task_wdt_reset();
}

// ---------- WiFi ----------
static void ensureWifi() {
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }
  SerialMon.print("[WiFi] conectando");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000UL) {
    watchdogFeed();
    delay(300);
    SerialMon.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    SerialMon.println();
    SerialMon.print("[WiFi] IP: ");
    SerialMon.println(WiFi.localIP());
  } else {
    SerialMon.println(" FAIL");
  }
}

// ---------- MQTT ----------
static String buildClientId() {
  return String("logan-adcwifi-") + String((uint32_t)ESP.getEfuseMac(), HEX);
}

static void publishStatus(const char *state) {
  if (!mqtt.connected()) {
    return;
  }
  mqtt.publish(TOPIC_STATUS, state, true);
  SerialMon.printf("[SYS] status=%s\n", state);
}

static bool tryConnectMQTT() {
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setKeepAlive(MQTT_KEEPALIVE_SEC);
  mqtt.setSocketTimeout(MQTT_SOCKET_TIMEOUT_SEC);

  String clientId = buildClientId();
  SerialMon.print("[MQTT] Connecting... ");

  bool ok = mqtt.connect(
      clientId.c_str(),
      MQTT_USER,
      MQTT_PASS,
      TOPIC_LWT,
      1,      // QoS 1 para el will
      true,   // retained
      LWT_OFFLINE);

  if (ok) {
    SerialMon.println("OK");
    mqtt.publish(TOPIC_LWT, LWT_ONLINE, true);
    publishStatus(bootStatusPublished ? "adc_wifi_reconnect" : "adc_wifi_boot");
    bootStatusPublished = true;
    mqttFailCount = 0;
    return true;
  }

  SerialMon.print("FAIL state=");
  SerialMon.println(mqtt.state());
  return false;
}

static void serviceMQTT() {
  if (mqtt.connected()) {
    return;
  }

  uint32_t now = millis();
  if (now < mqttNextAttemptMs) {
    return;
  }

  if (tryConnectMQTT()) {
    return;
  }

  mqttFailCount++;
  SerialMon.printf("[MQTT] fallos consecutivos=%u\n", mqttFailCount);

  if (mqttFailCount >= MQTT_FAILS_BEFORE_ESP_RESTART) {
    SerialMon.println("[MQTT] escalando -> reiniciar ESP32");
    delay(200);
    ESP.restart();
  }

  mqttNextAttemptMs = millis() + MQTT_RETRY_MS;
}

// ---------- ADC ----------
static void adcSetup() {
  analogReadResolution(12);
  // Atenuacion 11dB -> rango util ~0-3.1V (cubre los 2.96V del divisor).
  // (Core Arduino 3.x: el nombre equivalente es ADC_12db.)
  analogSetPinAttenuation(SENSE_PIN, ADC_11db);
}

static float readPinVolts() {
  uint32_t accMv = 0;
  for (int i = 0; i < SAMPLES; i++) {
    accMv += analogReadMilliVolts(SENSE_PIN);
    delay(2);
  }
  return (accMv / (float)SAMPLES) / 1000.0f;
}

// ---------- Publicaciones ----------
static void publishIgnition(float vPin, float vbus) {
  if (!mqtt.connected()) {
    return;
  }
  char payload[112];
  snprintf(payload, sizeof(payload),
           "{\"state\":\"%s\",\"pin\":%.3f,\"vbus\":%.2f,\"up\":%lu}",
           stateStr(ignState), vPin, vbus, (unsigned long)(millis() / 1000));
  mqtt.publish(TOPIC_IGNITION, payload, true);  // retained: ultimo estado visible
  SerialMon.printf("[IGN] %s pin=%.3fV vbus=%.2fV\n", stateStr(ignState), vPin, vbus);
}

static void publishAdc(float vPin, float vbus) {
  if (!mqtt.connected()) {
    return;
  }
  char payload[112];
  snprintf(payload, sizeof(payload),
           "{\"up\":%lu,\"pin\":%.3f,\"vbus\":%.2f,\"state\":\"%s\"}",
           (unsigned long)(millis() / 1000), vPin, vbus, stateStr(ignState));
  mqtt.publish(TOPIC_ADC, payload);  // QoS 0, no retained
}

// Maquina de estados con antirrebote. La zona media (0.6-2.5V) no cambia
// nada: limpia el candidato en curso (un dips breve no commuta a OFF).
static void serviceIgnition(float vPin, float vbus) {
  IgnState measured = ignState;
  if (vPin > PIN_ON_V) {
    measured = IGN_ON;
  } else if (vPin < PIN_OFF_V) {
    measured = IGN_OFF;
  }

  if (measured != ignState) {
    if (measured != candidate) {
      candidate = measured;
      candidateSinceMs = millis();
    }
    uint32_t need = (candidate == IGN_ON) ? ON_DEBOUNCE_MS : OFF_DEBOUNCE_MS;
    if (millis() - candidateSinceMs >= need) {
      ignState = candidate;
      publishIgnition(vPin, vbus);
    }
  } else {
    candidate = ignState;
  }
}

void setup() {
  SerialMon.begin(115200);
  delay(200);

  watchdogSetup();
  adcSetup();

  ensureWifi();
  tryConnectMQTT();

  SerialMon.println("[ADC_WIFI] listo: reportando por MQTT");
}

void loop() {
  watchdogFeed();

  if (WiFi.status() != WL_CONNECTED) {
    SerialMon.println("[WiFi] caido -> reconnect");
    ensureWifi();
  }

  serviceMQTT();
  mqtt.loop();

  static uint32_t lastSample = 0;
  static float lastPinV = 0.0f;
  static float lastVbusV = 0.0f;

  if (millis() - lastSample >= SAMPLE_PERIOD_MS) {
    lastSample = millis();
    lastPinV = readPinVolts();
    lastVbusV = lastPinV * DIVIDER_FACTOR;
    serviceIgnition(lastPinV, lastVbusV);
  }

  static uint32_t lastReport = 0;
  if (millis() - lastReport >= REPORT_PERIOD_MS) {
    lastReport = millis();
    publishAdc(lastPinV, lastVbusV);
  }

  delay(10);
}
