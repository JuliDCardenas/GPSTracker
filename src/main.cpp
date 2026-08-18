// ============================================================================
// GPS Tracker Logan - firmware principal (env: tracker)
//
// Telemetria GNSS por LTE hacia MQTT + sense de ignicion por ADC en GPIO9.
//
// El estado de la ignicion gobierna la cadencia de publicacion:
//   IGN_ON  en marcha  -> cada GPS_PERIOD_MS        (5 s)
//   IGN_ON  en ralenti -> cada IDLE_PERIOD_MS       (30 s)
//   IGN_OFF parqueado  -> cada PARKED_KEEPALIVE_MIN (20 min, configurable)
//
// Y ademas genera dos eventos discretos en las transiciones: engine_on y
// engine_off. El de apagado se publica con la ultima posicion valida guardada
// en RAM, de modo que el "aqui quedo parqueado" sale aunque el GNSS no tenga
// fix fresco en ese instante.
//
// NIVEL 1 DE AHORRO DE ENERGIA: aqui no duerme nada (ni el ESP32, ni el modem,
// ni el GNSS). Lo unico que baja es la cadencia de publicacion. Se eligio asi
// porque con este stack (TinyGSM + PubSubClient + LTE) volver a despertar
// cuesta 30-60 s de re-attach a plena potencia: pulso de PWRKEY, waitForNetwork,
// gprsConnect, mqtt.connect y el TTFF del GNSS (sin A-GNSS, porque AT+CAGPS da
// ERROR en el firmware B05V01_241206). Con intervalos cortos, dormir sale mas
// caro que quedarse despierto. El apagado del GNSS, el sleep del modem y el
// deep sleep del ESP32 (con wake por GPIO9, que es RTC-capable en el S3) quedan
// para despues de la prueba de autonomia con la 18650.
// ============================================================================

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

// Sense de ignicion: divisor 47k/68k sobre VBUS post-buck, con 510 ohm en
// serie, 100 nF en paralelo a R2 y Zener 3.6 V de clamp (catodo al pin).
// GPIO9 = ADC1_CH8. Se uso ADC1 a proposito: ADC2 queda inutilizable cuando
// la radio esta activa. GPIO1 quedo descartado por el pull-down parasitario
// del riel de la camara.
#define SENSE_PIN   9

// Bateria 18650 leida por el divisor que ya trae la placa. GPIO8 = ADC1_CH7,
// misma unidad de ADC que el sense, sin conflicto: se leen en secuencia.
#define BAT_ADC_PIN 8

// Prueba TCP previa a MQTT (solo para depuracion manual)
#define DEBUG_TCP_PROBE 0

// Puesto en 0, el firmware se comporta como antes de la integracion: cadencia
// fija de 5 s e ignition=1 siempre. Sirve para descartar al sense como causa
// de un problema en campo sin tener que revertir el commit.
#define IGNITION_SENSE_ENABLED 1

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

// Estado de ignicion retenido, mismo contrato que usaban los firmwares de
// prueba adc_sense_wifi / adc_sense_field, para no romper lo que ya escuche n8n.
const char TOPIC_IGNITION[] = "tracker/Lilygo/event/ignition";

// Voltaje de la 18650. Va en topic aparte y no dentro del CSV para no tocar el
// pipeline de Traccar. Cuando formalicemos el CSV v3 se podra mandar en la
// trama y mapearlo al parametro batt= del protocolo OsmAnd.
const char TOPIC_BATTERY[] = "tracker/Lilygo/sys/battery";

const char LWT_ONLINE[]  = "online";
const char LWT_OFFLINE[] = "offline";

// TLS client para el modem
TinyGsmClient netClient(modem);
PubSubClient mqtt(netClient);

// ---------- Contrato CSV ----------
// Formato publicado (12 campos):
//   v2,device,fix,lat,lon,speed,alt,vsat,acc,ignition,event,ts
//
// server/subscriberJsonOsmAnd.py ya lo parsea en normalize_v2_legacy_event(),
// asi que este cambio NO requiere tocar el servidor.
//
// OJO: en esta variante el campo "quality" no viaja. No se pierde informacion:
// el subscriber lo recalcula con build_quality() usando exactamente el mismo
// criterio que el firmware (campo vacio = invalido), asi que el resultado es
// identico al que se transmitia antes.
//
// Efecto util del lado del servidor: cuando ignition=0 o event != "-", el
// subscriber marca la trama como evento y salta el rate-limit, el filtro de
// movimiento minimo y el descarte por fix bajo. Por eso los keepalive de
// parqueo y el punto de apagado siempre llegan a Traccar.
const char EVENT_NONE[]       = "-";
const char EVENT_ENGINE_ON[]  = "engine_on";
const char EVENT_ENGINE_OFF[] = "engine_off";

// ---------- Timing ----------
static const uint32_t GPS_PERIOD_MS = 5000;   // ignicion ON y en movimiento
static const uint32_t IDLE_PERIOD_MS = 30000; // ignicion ON pero detenido

// Keepalive de parqueo. CONFIGURABLE: este es el numero a mover durante la
// prueba de autonomia con la 18650. 20 min = ~72 mensajes/dia.
static const uint32_t PARKED_KEEPALIVE_MIN = 20;

// Umbral para considerar que el vehiculo se esta moviendo.
// PENDIENTE: depende de la unidad real de AT+CGNSSINFO (ver nota de validacion
// GNSS mas abajo). A valores tan bajos da igual nudos que km/h, pero si algun
// dia se sube este umbral hay que resolver primero la unidad.
static const float MOVING_SPEED_KMH = 3.0f;

// Cuanto se sigue considerando "en marcha" despues del ultimo movimiento, para
// no saltar a cadencia lenta en cada semaforo.
static const uint32_t MOVING_HOLD_MS = 60000;

// Publicacion del voltaje de bateria con ignicion encendida. Con el vehiculo
// parqueado se publica junto al keepalive, no cada 5 min.
static const uint32_t BATTERY_PERIOD_MS = 300000;

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

// ---------- Sense de ignicion ----------
//
// CALIBRACION DEL DIVISOR (2026-08-17). Medido en banco con el buck alimentado
// desde el vehiculo:
//   salida del buck (multimetro) : 5.25 V
//   pin GPIO9 (multimetro)       : 2.74 V
//   pin GPIO9 (ADC del ESP32)    : 2.711 V  -> error de 27 mV (1.0 %), OK
//
// La relacion real del divisor es 2.74/5.25 = 0.5219, contra 0.5913 teorico de
// 47k/68k: una desviacion de -11.7 %. La causa mas probable es la fuga inversa
// del Zener 3.6 V, equivalente a unos 210 kOhm / 13 uA en paralelo con R2 sobre
// un divisor que solo trabaja con 40 uA.
//
// Por eso el factor NO es el teorico 1.6912 sino el empirico 5.25/2.711.
// Limitacion aceptada: al depender de una fuga, el VBUS reportado puede derivar
// con la temperatura. Los umbrales ON/OFF no se ven afectados porque estan
// definidos sobre el pin, no sobre VBUS.
//
// Pendiente de hardware (tarea aparte): bajar el divisor a 10k/12k y ajustar el
// pot del MP1584 de 5.25 V a 5.05-5.10 V. Ahi habra que recalibrar este factor
// y bajar el umbral ON a 2.2 V.
static const float DIVIDER_FACTOR = 1.9366f;

static const uint8_t  ADC_SAMPLES     = 32;
static const uint8_t  BAT_SAMPLES     = 16;
static const float    PIN_ON_V        = 2.5f;   // ~4.83 V de VBUS
static const float    PIN_OFF_V       = 0.6f;   // residual medido: 0.018-0.04 V
static const uint32_t ON_DEBOUNCE_MS  = 3000;
static const uint32_t OFF_DEBOUNCE_MS = 20000;  // absorbe la caida del crank
static const uint32_t IGN_SAMPLE_MS   = 250;

// Si el pin se queda en tierra de nadie (entre OFF y ON) mas de este tiempo,
// algo pasa con el divisor: soldadura fria, cable suelto, Zener en corto.
static const uint32_t IGN_MIDZONE_WARN_MS = 120000;

// Factor del divisor de bateria de la placa (teorico 2.0 para 100k/100k).
// TODO CALIBRAR: medir con multimetro en los bornes de la 18650 y ajustar,
// exactamente como hubo que hacerlo con el divisor de ignicion, donde el
// teorico 1.6912 resulto ser 1.9366 en la vida real. Mientras no se calibre,
// este voltaje sirve para ver tendencia, no para disparar alertas finas.
static const float BAT_DIVIDER_FACTOR = 2.0f;

// ---------- Validación GNSS ----------
// PENDIENTE: verificar la unidad real que entrega AT+CGNSSINFO. La evidencia
// de campo (Traccar coincide con el odómetro del vehículo) apunta a que el
// módem reporta NUDOS y no km/h, por lo que este umbral estaría filtrando en
// realidad a ~333 km/h. No cambiar hasta confirmarlo con más mediciones.
static const float MAX_VALID_SPEED_KMH = 180.0f;
static const float MIN_VALID_ALTITUDE_M = -9990.0f;
static const int MIN_VALID_SATELLITES = 5;
static const float MAX_VALID_HDOP = 2.5f;

// Bitmask de calidad heredado del CSV v2 de 11 campos. Se conserva porque el
// firmware sigue usando el mismo criterio para decidir si un campo viaja vacio,
// aunque el valor ya no se transmita (lo recalcula el subscriber).
// bit 0 = altitud válida
// bit 1 = velocidad válida
static const uint8_t GPS_QUALITY_ALT_VALID = 1;
static const uint8_t GPS_QUALITY_SPEED_VALID = 2;

// ---------- Estado de conexión ----------
static uint8_t mqttFailCount = 0;
static uint32_t mqttNextAttemptMs = 0;
static uint32_t mqttRetryDelayMs = MQTT_RETRY_BASE_MS;
static bool bootStatusPublished = false;

// ---------- Estado de ignicion ----------
// IGN_UNKNOWN es el estado de arranque y tambien el de la zona media. Se trata
// como encendido a efectos de cadencia (fail-safe: es preferible publicar de
// mas que perderle el rastro al carro por un sense que no resolvio).
enum IgnState : uint8_t { IGN_UNKNOWN = 0, IGN_ON = 1, IGN_OFF = 2 };

static IgnState ignState = IGN_UNKNOWN;
static IgnState ignCandidate = IGN_UNKNOWN;
static uint32_t ignCandidateSinceMs = 0;
static uint32_t ignLastSampleMs = 0;
static float ignLastPinV = 0.0f;
static uint32_t midzoneSinceMs = 0;
static bool midzoneWarned = false;

enum PendingEvent : uint8_t { EV_NONE = 0, EV_ENGINE_ON = 1, EV_ENGINE_OFF = 2 };
static PendingEvent pendingEvent = EV_NONE;

// ---------- Ultimo punto valido ----------
// Se cachea en RAM para poder publicar el evento de apagado y los keepalive de
// parqueo sin depender de que el GNSS tenga fix fresco en ese momento.
struct GpsPoint {
  bool valid;
  uint8_t fix;
  float lat;
  float lon;
  float speed;
  float alt;
  float acc;
  int vsat;
  bool speedValid;
  bool altValid;
  char ts[24];
};

static GpsPoint lastValidPoint = {};
static uint32_t lastMovementMs = 0;
static uint32_t lastPublishMs = 0;
static uint32_t lastBatteryMs = 0;
static bool batteryEverPublished = false;

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

// Detector de parseo corrupto del GNSS.
//
// De vez en cuando el modem entrega una respuesta +CGNSSINFO truncada o mezclada
// con un URC, y TinyGSM la da por buena. En banco (2026-08-18, 03:47 UTC) se
// vieron dos tramas seguidas con fecha "-7999-34-00T00:59:00Z", velocidad y
// altitud invalidas, y un HDOP de 0.87 cuando sus vecinas estaban en 0.65.
// Ahi la lat/lon salio bien de casualidad; si el parseo se corre un campo,
// publicariamos una coordenada basura y Traccar dibuja el carro en otro pais.
// La fecha es el canario mas confiable del parseo, y con cadencia de 5 s
// descartar la trama completa no cuesta nada.
static bool isGpsTimeValid(int year, int month, int day, int hour, int minute, int second) {
  return year >= 2024 && year <= 2099 &&
         month >= 1 && month <= 12 &&
         day >= 1 && day <= 31 &&
         hour >= 0 && hour <= 23 &&
         minute >= 0 && minute <= 59 &&
         second >= 0 && second <= 59;
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

// ---------- ADC ----------
static void adcSetup() {
  analogReadResolution(12);
  // 11 dB para cubrir el rango util del divisor (0-3.1 V aprox).
  analogSetPinAttenuation(SENSE_PIN, ADC_11db);
  analogSetPinAttenuation(BAT_ADC_PIN, ADC_11db);
}

static float readPinVolts() {
  uint32_t acc_mv = 0;
  for (uint8_t i = 0; i < ADC_SAMPLES; i++) {
    acc_mv += analogReadMilliVolts(SENSE_PIN);
  }
  return (acc_mv / (float)ADC_SAMPLES) / 1000.0f;
}

static float readBatteryVolts() {
  uint32_t acc_mv = 0;
  for (uint8_t i = 0; i < BAT_SAMPLES; i++) {
    acc_mv += analogReadMilliVolts(BAT_ADC_PIN);
  }
  return ((acc_mv / (float)BAT_SAMPLES) / 1000.0f) * BAT_DIVIDER_FACTOR;
}

static bool isIgnitionOff() {
#if IGNITION_SENSE_ENABLED
  return ignState == IGN_OFF;
#else
  return false;
#endif
}

static uint8_t ignitionField() {
  return isIgnitionOff() ? 0 : 1;
}

// Muestrea el pin y resuelve el estado con antirrebote.
//
// El antirrebote se mide con deltas de millis(), NO contando muestras. Eso
// importa porque el loop se bloquea de a ratos en llamadas AT y de red
// (waitForNetwork puede tardar 60 s). Un bloqueo solo retrasa la deteccion,
// no la rompe: al volver, la funcion ve el pin y el tiempo transcurrido.
static void serviceIgnition() {
#if IGNITION_SENSE_ENABLED
  uint32_t now = millis();
  if (now - ignLastSampleMs < IGN_SAMPLE_MS) {
    return;
  }
  ignLastSampleMs = now;

  float pinV = readPinVolts();
  ignLastPinV = pinV;

  IgnState observed;
  if (pinV >= PIN_ON_V) {
    observed = IGN_ON;
  } else if (pinV <= PIN_OFF_V) {
    observed = IGN_OFF;
  } else {
    observed = IGN_UNKNOWN;  // zona media
  }

  if (observed == IGN_UNKNOWN) {
    if (midzoneSinceMs == 0) {
      midzoneSinceMs = now;
    } else if (!midzoneWarned && (now - midzoneSinceMs) >= IGN_MIDZONE_WARN_MS) {
      SerialMon.printf("[IGN] zona media sostenida pin=%.3fV -> revisar divisor\n", pinV);
      publishStatus("ign_sense_midzone");
      midzoneWarned = true;
    }
    return;  // conserva el ultimo estado conocido
  }

  midzoneSinceMs = 0;
  midzoneWarned = false;

  if (observed != ignCandidate) {
    ignCandidate = observed;
    ignCandidateSinceMs = now;
    return;
  }

  if (observed == ignState) {
    return;
  }

  uint32_t needed = (observed == IGN_ON) ? ON_DEBOUNCE_MS : OFF_DEBOUNCE_MS;
  if ((now - ignCandidateSinceMs) < needed) {
    return;
  }

  ignState = observed;
  pendingEvent = (ignState == IGN_ON) ? EV_ENGINE_ON : EV_ENGINE_OFF;
  SerialMon.printf("[IGN] %s pin=%.3fV vbus=%.2fV\n",
                   (ignState == IGN_ON) ? "ON" : "OFF", pinV, pinV * DIVIDER_FACTOR);
#endif
}

// Periodo de publicacion segun el estado actual.
static uint32_t currentPeriodMs() {
  if (isIgnitionOff()) {
    return PARKED_KEEPALIVE_MIN * 60000UL;
  }
  // lastMovementMs se inicializa en setup() y se refresca en cada engine_on, no
  // solo cuando hay movimiento real. Si dependiera unicamente del movimiento, un
  // vehiculo encendido y quieto (un trancon, o el banco de pruebas) nunca
  // bajaria a cadencia lenta: se detecto asi en banco el 2026-08-18, publicando
  // cada 5 s durante un minuto entero con velocidad 0.00.
  if ((millis() - lastMovementMs) > MOVING_HOLD_MS) {
    return IDLE_PERIOD_MS;
  }
  return GPS_PERIOD_MS;
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
  //
  // El LWT NO se mapea a ignition=false en el subscriber (modo "event"):
  // una caida de LTE no significa que el motor se apago. El estado real de
  // ignicion es el que publica este firmware desde el ADC.
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
    //
    // Sirve de forense: si despues de una noche corriendo el retained sigue en
    // "boot", el ESP32 no se reinicio; si dice "mqtt_reconnected", hubo caidas
    // de enlace pero el firmware sobrevivio.
    publishStatus(bootStatusPublished ? "mqtt_reconnected" : "boot");
    bootStatusPublished = true;

#if IGNITION_SENSE_ENABLED
    // Reafirma el estado de ignicion al reconectar, por si el retained del
    // broker quedo viejo tras una caida larga.
    if (ignState != IGN_UNKNOWN) {
      mqtt.publish(TOPIC_IGNITION, (ignState == IGN_ON) ? "on" : "off", true);
    }
#endif

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
  SerialMon.printf("[BAT] %s V (factor %.2f SIN CALIBRAR)\n", buf, BAT_DIVIDER_FACTOR);
}

void setup() {
  SerialMon.begin(115200);
  delay(200);

  watchdogSetup();

  adcSetup();

  // Arranca el reloj del ralenti desde el boot. Sin esto, un vehiculo encendido
  // que nunca supera MOVING_SPEED_KMH se queda para siempre en cadencia rapida.
  lastMovementMs = millis();

  // Lectura informativa de arranque. El estado real se resuelve en el loop con
  // su antirrebote: hasta entonces IGN_UNKNOWN se comporta como encendido.
  float bootPinV = readPinVolts();
  SerialMon.printf("[IGN] boot pin=%.3fV vbus=%.2fV (sense %s)\n",
                   bootPinV, bootPinV * DIVIDER_FACTOR,
                   IGNITION_SENSE_ENABLED ? "activo" : "desactivado");
  SerialMon.printf("[BAT] boot %.2fV (factor %.2f SIN CALIBRAR)\n",
                   readBatteryVolts(), BAT_DIVIDER_FACTOR);

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

  // Modo GNSS multi-constelacion: 15 = GPS + GLONASS + GALILEO + BDS.
  // AT+CGNSSMODE solo es aceptado por el modem cuando el GNSS ya esta
  // encendido (verificado en banco 2026-08-16, firmware SIM767XM5_B05V01_241206),
  // por eso va DESPUES de enableGPS(). El valor persiste en la NVRAM del
  // modem (sobrevive reflashes del ESP32 y apagados), pero se fija en cada
  // boot para que el comportamiento sea deterministico.
  if (modem.setGPSMode(15)) {
    SerialMon.println("GNSS mode 15 (GPS+GLONASS+GALILEO+BDS)");
  } else {
    SerialMon.println("GNSS mode 15 FAIL (no critico, sigue en modo actual)");
  }
}

void loop() {
  watchdogFeed();

  // El sense va de primero y en cada vuelta. El loop gira cada ~10 ms, asi que
  // la cadencia efectiva del ADC es mejor que IGN_SAMPLE_MS salvo cuando una
  // llamada bloqueante (waitForNetwork, mqtt.connect, getGPS) se toma el hilo.
  // No hace falta un segundo nucleo: 32 muestras cada 250 ms son microsegundos
  // de CPU, y repartir esto entre cores obligaria a sincronizar el estado con
  // el cliente MQTT, que no es thread-safe.
  serviceIgnition();

  // Mantener sesión MQTT viva
  if (!modem.isNetworkConnected() || !modem.isGprsConnected()) {
    SerialMon.println("[NET] down -> reconnect");
    ensureLTE();
  }

  serviceMQTT();
  mqtt.loop();

  serviceEvents();
  serviceTelemetry();
  serviceBattery();

  delay(10);
}
