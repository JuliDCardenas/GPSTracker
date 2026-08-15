// PRUEBA DE CAMBIO DE RAMAS


#include <Arduino.h>
#include <esp_task_wdt.h>


#define TINY_GSM_RX_BUFFER 1024
#define SerialMon Serial
#define TINY_GSM_DEBUG SerialMon

// Modem (ajusta si tu fork pide otra macro)
#define TINY_GSM_MODEM_SIM7670G
#include <TinyGsmClient.h>
#include <PubSubClient.h>

// Credenciales locales. Copia include/secrets.example.h como
// include/secrets.h y completa los valores reales.
// include/secrets.h esta en .gitignore y no debe subirse al repositorio.
#include "secrets.h"

// ---------- Pines ----------
#define MODEM_BAUDRATE 115200
#define MODEM_TX_PIN        4
#define MODEM_RX_PIN        5
#define MODEM_DTR_PIN       7
#define BOARD_PWRKEY_PIN    46

#define MODEM_GPS_ENABLE_GPIO   1
#define MODEM_GPS_ENABLE_LEVEL  1

#define MODEM_POWERON_PULSE_WIDTH_MS 1000

// Prueba TCP previa a MQTT (solo para depuracion manual)
#define DEBUG_TCP_PROBE 0

HardwareSerial SerialAT(1);
TinyGsm modem(SerialAT);

// ---------- Identidad del dispositivo ----------
// APN, APN_USER, APN_PASS, MQTT_HOST, MQTT_PORT, MQTT_USER y MQTT_PASS
// se definen en include/secrets.h

const char DEVICE_ID[] = "Lilygo";
const char TOPIC_TELEMETRY[] = "tracker/Lilygo/telemetria";

// Topics de sistema (bajo volumen, QoS 1 / retained donde aplica)
const char TOPIC_LWT[]    = "tracker/Lilygo/sys/lwt";
const char TOPIC_STATUS[] = "tracker/Lilygo/sys/status";

const char LWT_ONLINE[]  = "online";
const char LWT_OFFLINE[] = "offline";

// TLS client para el modem
TinyGsmClient netClient(modem);
PubSubClient mqtt(netClient);

// ---------- Timing ----------
static const uint32_t GPS_PERIOD_MS = 5000;  // Acá definimos la latencía de envío de los datos

// Keepalive amplio: LTE tiene microcortes y cambios de celda.
// Con 60 s el broker tolera latencia sin cortar la sesión.
static const uint16_t MQTT_KEEPALIVE_SEC = 60;
static const uint16_t MQTT_SOCKET_TIMEOUT_SEC = 15;

// Backoff de reconexión MQTT (no bloqueante)
static const uint32_t MQTT_RETRY_BASE_MS = 2000;
static const uint32_t MQTT_RETRY_MAX_MS  = 30000;

// Escalamiento de recuperación por fallos consecutivos
static const uint8_t MQTT_FAILS_BEFORE_LTE_RECONNECT = 3;
static const uint8_t MQTT_FAILS_BEFORE_MODEM_RESTART = 6;
static const uint8_t MQTT_FAILS_BEFORE_ESP_RESTART   = 10;

// Watchdog: si el firmware se cuelga, el ESP32 se reinicia solo.
static const uint32_t WDT_TIMEOUT_SEC = 120;

// ---------- Validación GNSS ----------
// PENDIENTE: verificar la unidad real que entrega AT+CGNSSINFO. La evidencia
// de campo (Traccar coincide con el odómetro del vehículo) apunta a que el
// módem reporta NUDOS y no km/h, por lo que este umbral estaría filtrando en
// realidad a ~333 km/h. No cambiar hasta confirmarlo con más mediciones.
static const float MAX_VALID_SPEED_KMH = 180.0f;
static const float MIN_VALID_ALTITUDE_M = -9990.0f;
static const int MIN_VALID_SATELLITES = 5;
static const float MAX_VALID_HDOP = 2.5f;

// Bitmask de calidad para CSV v2:
// bit 0 = altitud válida
// bit 1 = velocidad válida
// Valores posibles:
// 0 = altitud inválida + velocidad inválida
// 1 = altitud válida + velocidad inválida
// 2 = altitud inválida + velocidad válida
// 3 = altitud válida + velocidad válida
static const uint8_t GPS_QUALITY_ALT_VALID = 1;
static const uint8_t GPS_QUALITY_SPEED_VALID = 2;

// ---------- Estado de conexión ----------
static uint8_t mqttFailCount = 0;
static uint32_t mqttNextAttemptMs = 0;
static uint32_t mqttRetryDelayMs = MQTT_RETRY_BASE_MS;
static bool bootStatusPublished = false;

static bool isGpsPositionValid(uint8_t fix, float lat, float lon, int satellites, float hdop) {
  bool hasFix = fix >= 2;  // 2D/3D fix
  bool hasValidPosition = lat >= -90.0f && lat <= 90.0f && lon >= -180.0f && lon <= 180.0f && !(lat == 0.0f && lon == 0.0f);
  bool hasEnoughSatellites = satellites >= MIN_VALID_SATELLITES;
  bool hasGoodPrecision = hdop > 0.0f && hdop <= MAX_VALID_HDOP;

  return hasFix && hasValidPosition && hasEnoughSatellites && hasGoodPrecision;
}

static bool isGpsSpeedValid(float speedKmh) {
  return speedKmh >= 0.0f && speedKmh <= MAX_VALID_SPEED_KMH;
}

static bool isGpsAltitudeValid(float altitudeM) {
  return altitudeM > MIN_VALID_ALTITUDE_M;
}

static uint8_t buildGpsQuality(float speedKmh, float altitudeM) {
  uint8_t quality = 0;

  if (isGpsAltitudeValid(altitudeM)) {
    quality |= GPS_QUALITY_ALT_VALID;
  }
  if (isGpsSpeedValid(speedKmh)) {
    quality |= GPS_QUALITY_SPEED_VALID;
  }

  return quality;
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

// ---------- Helpers ----------
static String buildClientId() {
  return String("logan-") + String((uint32_t)ESP.getEfuseMac(), HEX);
}

static void publishStatus(const char *state) {
  if (!mqtt.connected()) {
    return;
  }
  // QoS 1 no está disponible en publish() de PubSubClient para salida,
  // pero sí retained. Los mensajes de sistema son pocos y van retenidos
  // para que el último estado quede siempre disponible en el broker.
  mqtt.publish(TOPIC_STATUS, state, true);
  SerialMon.printf("[SYS] status=%s\n", state);
}

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
    watchdogFeed();
    delay(1000);
  }
  SerialMon.println("AT OK");
}

static void ensureLTE() {
  watchdogFeed();

  SerialMon.print("Waiting for network...");
  if (!modem.waitForNetwork(60000L)) {
    SerialMon.println(" FAIL");
    return;
  }
  SerialMon.println(" OK");

  watchdogFeed();

  if (modem.isGprsConnected()) {
    SerialMon.println("GPRS ya conectado");
  } else {
    SerialMon.print("Connecting GPRS/APN...");
    if (!modem.gprsConnect(APN, APN_USER, APN_PASS)) {
      SerialMon.println(" FAIL");
      return;
    }
    SerialMon.println(" OK");
  }

  IPAddress ip = modem.localIP();
  SerialMon.print("IP: ");
  SerialMon.println(ip);
}

static void restartModem() {
  SerialMon.println("[NET] reiniciando modem...");
  watchdogFeed();
  modem.gprsDisconnect();
  delay(500);
  modem.restart();
  watchdogFeed();
  waitForAT();
  ensureLTE();
}

// Intento único de conexión MQTT. No bloquea el loop.
// Devuelve true si quedó conectado.
static bool tryConnectMQTT() {
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setKeepAlive(MQTT_KEEPALIVE_SEC);
  mqtt.setSocketTimeout(MQTT_SOCKET_TIMEOUT_SEC);

  String clientId = buildClientId();

#if DEBUG_TCP_PROBE
  SerialMon.printf("[TCP] probe %s:%d ... ", MQTT_HOST, MQTT_PORT);
  if (netClient.connect(MQTT_HOST, MQTT_PORT)) {
    SerialMon.println("OK");
    netClient.stop();
  } else {
    SerialMon.println("FAIL");
  }
#endif

  SerialMon.print("[MQTT] Connecting... ");

  // Last Will Testament: si el dispositivo desaparece sin cerrar sesión,
  // el broker publica "offline" retenido en TOPIC_LWT.
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
    // TOPIC_STATUS es retained: el último publicado es el que queda en el
    // broker. Por eso "boot" se publica aquí, en la primera conexión de la
    // sesión, y no después en setup(), donde sobrescribiría el estado real.
    publishStatus(bootStatusPublished ? "mqtt_reconnected" : "boot");
    bootStatusPublished = true;
    mqttFailCount = 0;
    mqttRetryDelayMs = MQTT_RETRY_BASE_MS;
    return true;
  }

  SerialMon.print("FAIL state=");
  SerialMon.println(mqtt.state());
  return false;
}

// Gestiona reconexión con backoff y escalamiento de recuperación.
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

  if (mqttFailCount == MQTT_FAILS_BEFORE_LTE_RECONNECT) {
    SerialMon.println("[MQTT] escalando -> reconectar LTE");
    ensureLTE();
  } else if (mqttFailCount == MQTT_FAILS_BEFORE_MODEM_RESTART) {
    SerialMon.println("[MQTT] escalando -> reiniciar modem");
    restartModem();
  } else if (mqttFailCount >= MQTT_FAILS_BEFORE_ESP_RESTART) {
    SerialMon.println("[MQTT] escalando -> reiniciar ESP32");
    delay(200);
    ESP.restart();
  }

  // Backoff exponencial acotado
  mqttRetryDelayMs = mqttRetryDelayMs * 2;
  if (mqttRetryDelayMs > MQTT_RETRY_MAX_MS) {
    mqttRetryDelayMs = MQTT_RETRY_MAX_MS;
  }
  mqttNextAttemptMs = millis() + mqttRetryDelayMs;
}

void setup() {
  SerialMon.begin(115200);
  delay(200);

  watchdogSetup();

  modemPowerOn();
  SerialAT.begin(MODEM_BAUDRATE, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);

  waitForAT();

  // 1) LTE up
  ensureLTE();

  // 2) MQTT up (primer intento; el loop se encarga de reintentar).
  //    El status "boot" lo publica tryConnectMQTT() al conectar.
  tryConnectMQTT();

  // 3) GPS on
  SerialMon.println("Enabling GPS...");
  while (!modem.enableGPS(MODEM_GPS_ENABLE_GPIO, MODEM_GPS_ENABLE_LEVEL)) {
    SerialMon.print(".");
    watchdogFeed();
    delay(500);
  }
  SerialMon.println("\nGPS Enabled");
  modem.setGPSBaud(115200);
}

void loop() {
  watchdogFeed();

  // Mantener sesión MQTT viva
  if (!modem.isNetworkConnected() || !modem.isGprsConnected()) {
    SerialMon.println("[NET] down -> reconnect");
    ensureLTE();
  }

  serviceMQTT();
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

    bool positionValid = isGpsPositionValid(fix, lat, lon, vsat, acc);
    if (!positionValid) {
      SerialMon.printf("[GPS] posición inválida -> no se publica fix=%u lat=%.6f lon=%.6f speed=%.2f alt=%.1f sats=%d hdop=%.2f\n",
                       fix, lat, lon, speed, alt, vsat, acc);
      return;
    }

    uint8_t quality = buildGpsQuality(speed, alt);
    bool speedValid = (quality & GPS_QUALITY_SPEED_VALID) != 0;
    bool altValid = (quality & GPS_QUALITY_ALT_VALID) != 0;

    char speedField[16] = "";
    char altField[16] = "";

    if (speedValid) {
      snprintf(speedField, sizeof(speedField), "%.2f", speed);
    }
    if (altValid) {
      snprintf(altField, sizeof(altField), "%.1f", alt);
    }

    if (!speedValid || !altValid) {
      SerialMon.printf("[GPS] posición válida con datos parciales quality=%u speed=%.2f alt=%.1f sats=%d hdop=%.2f\n",
                       quality, speed, alt, vsat, acc);
    }

    char payload[256];
    snprintf(payload, sizeof(payload),
             "v2,%s,%u,%.6f,%.6f,%s,%s,%d,%.2f,%u,%04d-%02d-%02dT%02d:%02d:%02dZ",
             DEVICE_ID, fix, lat, lon, speedField, altField, vsat, acc, quality,
             year, month, day, hour, min, sec);

    if (!mqtt.connected()) {
      SerialMon.println("[PUB] omitido: MQTT desconectado");
      return;
    }

    // Telemetría GPS en QoS 0: alto volumen, perder un punto no es crítico.
    bool pubOk = mqtt.publish(TOPIC_TELEMETRY, payload);
    SerialMon.printf("[PUB] %s topic=%s payload=%s\n", pubOk ? "OK" : "FAIL", TOPIC_TELEMETRY, payload);
  }

  delay(10);
}
