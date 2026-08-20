// ============================================================================
// GPS Tracker Logan - firmware principal (env: tracker)
//
// Telemetria GNSS por LTE hacia MQTT + sense de ignicion por ADC en GPIO9.
//
// El estado de la ignicion gobierna la cadencia de publicacion:
//   IGN_ON  en marcha  -> cada GPS_PERIOD_MS        (5 s)
//   IGN_ON  en ralenti -> cada IDLE_PERIOD_MS       (30 s)
//   IGN_OFF parqueado  -> engine_off y luego deep sleep (Nivel 2): repaso de
//                         ignicion cada 30 s y un pulso de bateria/posicion al dia
//
// Y ademas genera dos eventos discretos en las transiciones: engine_on y
// engine_off. El de apagado se publica con la ultima posicion valida guardada
// en la cache, de modo que el "aqui quedo parqueado" sale aunque el GNSS no
// tenga fix fresco en ese instante.
//
// NIVEL 1 DE AHORRO DE ENERGIA: aqui no duerme nada (ni el ESP32, ni el modem,
// ni el GNSS). Lo unico que baja es la cadencia de publicacion. Se eligio asi
// porque con este stack (TinyGSM + PubSubClient + LTE) volver a despertar
// cuesta 30-60 s de re-attach a plena potencia: pulso de PWRKEY, waitForNetwork,
// gprsConnect, mqtt.connect y el TTFF del GNSS (sin A-GNSS, porque AT+CAGPS da
// ERROR en el firmware B05V01_241206). Con intervalos cortos, dormir sale mas
// caro que quedarse despierto.
//
// MEDIDO (noche del 2026-08-17 al 18, 8 h 22 min parqueado con keepalive de
// 20 min): 25 de 25 keepalives entregados, cero reconexiones MQTT, y la 18650
// bajo de ~4.11 V a 3.79 V, unos 38 mV/h. Autonomia estimada del Nivel 1:
// 15-18 h. Con solo 25 transmisiones en 8 horas, el intervalo del keepalive es
// irrelevante para el consumo: lo que cuesta es tener el modem enganchado y el
// GNSS encendido. El siguiente salto real es el Nivel 2 (AT+CGNSSPWR=0 entre
// keepalives y modem dormido por DTR), no publicar menos seguido.
//
// NIVEL 2 DE AHORRO DE ENERGIA: parqueado, el firmware publica el engine_off,
// apaga el GNSS (AT+CGNSSPWR=0), duerme el modem por DTR (AT+CSCLK=1; conserva
// el registro LTE y las efemerides) y pone el ESP32 en deep sleep. Despierta
// por ext0 si la ignicion sube y, como red de seguridad, cada 30 s por
// temporizador. Una vez al dia (PARKED_PULSE_S) despierta el modem, publica
// bateria + posicion y vuelve a dormir. El corte por bajo voltaje
// (BAT_CUTOFF_V, con histeresis BAT_RECOVER_V) apaga TODO con AT+CPOF y solo
// rearma con la celda recuperada o con VBUS presente. El guardian de arranque
// bloquea cualquier boot sin VBUS por debajo del corte, sin encender el
// modem: eso mata el bucle de brownout del 2026-08-18 (10 reconexiones en
// 108 s, muerte a 2.37 V).
//
// OJO CON EL RETRASO DE DETECCION: los 30 s del repaso NO son la latencia de
// deteccion de la ignicion. El despertar real lo hace ext0, que es una
// interrupcion de hardware en GPIO9 y es instantanea. El temporizador existe
// como red de seguridad (si ext0 no se pudo armar) y para vigilar el voltaje
// de la celda mientras duerme. Al encender el carro, lo que se demora son el
// antirrebote (3 s) y el re-attach de LTE+MQTT (~5-15 s), no el sondeo.
//
// REGLA DE ORO DEL NIVEL 2 (aprendida a golpes, 2026-08-20): de esta funcion
// no se sale nunca sin una fuente de despertar armada. Un deep sleep sin ext0
// y sin temporizador no es un ahorro de energia, es un ladrillo que solo revive
// desconectando la bateria. Ver pmDeepSleep().
//
// SEGUNDA REGLA (misma noche, mismo precio): si el firmware puede quedarse
// varios minutos sin publicar y sin imprimir, entonces un cuelgue y una espera
// normal se ven EXACTAMENTE igual, y se depura a ciegas adivinando. Todo estado
// que dure mas de unos segundos tiene que anunciarse. Ver el log periodico del
// loop y TEST_DISABLE_SLEEP.
//
// TERCERA REGLA (la que costo toda la noche): ninguna decision irreversible se
// toma con una sola muestra del ADC. Las primeras conversiones tras el arranque
// son basura y casi dejaron esta rama por inservible. Ver adcSetup() y
// pmBootGuard().
// ============================================================================

#include <Arduino.h>
#include <esp_task_wdt.h>
#include <esp_sleep.h>
#include <driver/gpio.h>


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
//
// OJO AL DEPURAR CON mosquitto_sub: este topic, TOPIC_LWT, TOPIC_STATUS y
// TOPIC_BATTERY son retained, asi que el broker te los entrega de golpe al
// suscribirte. Esas primeras lineas son la foto de la sesion ANTERIOR, no
// actividad nueva. El 2026-08-20 eso costo un diagnostico equivocado: se leyo
// un "ignition on" retenido como si fuera un evento del momento.
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
//
// VALIDADO en la noche del 2026-08-17 al 18: con el motor encendido y el equipo
// quieto, el servidor descarto cada trama con "Skip: no_move_0.2m"; parqueado,
// tramas practicamente identicas pasaron las 25 veces gracias al bypass por
// ignition=0. Sin ese bypass no habria quedado ni un punto de la noche.
const char EVENT_NONE[]       = "-";
const char EVENT_ENGINE_ON[]  = "engine_on";
const char EVENT_ENGINE_OFF[] = "engine_off";

// ---------- Timing ----------
static const uint32_t GPS_PERIOD_MS = 5000;   // ignicion ON y en movimiento
static const uint32_t IDLE_PERIOD_MS = 30000; // ignicion ON pero detenido

// Keepalive de parqueo del Nivel 1. Con el Nivel 2 ya no se alcanza ese punto
// del loop (el firmware duerme al minuto de apagado), pero se conserva la
// constante por si hay que volver atras.
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
// RECONFIRMADO el 2026-08-20, tres dias despues: el firmware reporto
// pin=2.721V -> vbus=5.27V con el mismo montaje. La calibracion aguanta.
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

// Conversiones que se descartan en adcSetup() antes de dar el ADC por usable,
// y pausa entre ellas. Ver el comentario largo de adcSetup(): sin esto, las
// primeras lecturas son basura y el guardian de arranque decide sobre ruido.
static const uint8_t  ADC_WARMUP_READS  = 8;
static const uint32_t ADC_WARMUP_STEP_MS = 5;

// Si el pin se queda en tierra de nadie (entre OFF y ON) mas de este tiempo,
// algo pasa con el divisor: soldadura fria, cable suelto, Zener en corto.
static const uint32_t IGN_MIDZONE_WARN_MS = 120000;

// Factor del divisor de bateria de la placa (100k/100k = 2.0 teorico).
//
// CALIBRADO (2026-08-18). Medicion en banco con la 18650 ya descargada tras una
// noche de keepalive:
//   bornes de la bateria (multimetro) : 3.79 V
//   reportado por el firmware         : 3.78 V
// Error de 10 mV (0.26 %), dentro del error propio del ADC. El factor teorico
// queda confirmado y NO requiere ajuste.
//
// Contraste util con el divisor de ignicion, donde el teorico 1.6912 resulto ser
// 1.9366 en la vida real: alli hay un Zener fugando en paralelo con R2 sobre un
// divisor de solo 40 uA. Aqui el divisor de la placa esta limpio, sin nada en
// paralelo, y por eso la teoria si acierta.
static const float BAT_DIVIDER_FACTOR = 2.0f;

// ---------- Nivel 2 + corte por bajo voltaje ----------
// Umbrales de la 18650. La secuencia de muerte del 2026-08-18 (10 reconexiones
// en 108 s y ultimo voltaje 2.37 V) es exactamente lo que este corte viene a
// impedir: a 3.50 V se apaga todo con AT+CPOF y solo se rearma por encima de
// 3.80 V o con VBUS presente.
#define BAT_WARN_V        3.60f
#define BAT_CUTOFF_V      3.50f
#define BAT_RECOVER_V     3.80f   // histeresis de rearranque
#define BAT_CUTOFF_N      5       // lecturas consecutivas bajo corte
#define BAT_SAMPLE_MS     10000UL // 5 x 10 s = 50 s sostenidos

// Piso de PLAUSIBILIDAD, que no es lo mismo que un umbral de corte.
//
// Por debajo de esto la lectura no describe una celda descargada: describe una
// celda que no se puede medir, o un ADC que todavia no se ha asentado. El
// 2026-08-20 esta guarda fue lo unico que impidio que el equipo se durmiera una
// hora con la celda a 4.19 V, porque la primera lectura del boot dijo 0.84 V.
//
// El limite va en 2.0 V a proposito: los 2.37 V de la muerte del 2026-08-18
// quedan por encima y siguen siendo detectados como lo que son, una celda
// agonizando. Una 18650 real nunca reporta 1.9 V por este divisor.
#define BAT_PLAUSIBLE_MIN_V 2.00f

// Parqueo profundo: el ESP32 duerme y repasa la ignicion cada 30 s. El ext0 la
// detecta al instante si el margen del divisor alcanza (~2.62 V contra un
// umbral digital de ~2.48 V); el temporizador es la red de seguridad.
//
// PARKED_POLL_S no es la latencia de deteccion (eso lo hace ext0, instantaneo):
// es cada cuanto se revisa la bateria mientras duerme. Se puede subir a 60 o
// 120 s para ahorrar despertares una vez el Nivel 2 este cerrado; cada repaso
// es un arranque completo del ESP32.
#define PARKED_POLL_S     30UL
#define PARKED_PULSE_S    (24UL * 3600UL)
#define SETTLE_MS         800     // dejar salir los MQTT antes de desconectar

// Red de seguridad contra el ladrillo: si por cualquier razon no se pudo armar
// ext0, se duerme con temporizador en vez de quedarse dormido para siempre.
#define SLEEP_FALLBACK_S  30UL

// Con la celda bajo el piso y sin VBUS no se enciende nada, pero se repasa una
// vez por hora para poder notar la recuperacion y volver a la vida solo.
#define GUARD_RECHECK_S   3600UL

// Parqueo desde el loop: minuto de gracia para terminar el boot y publicar el
// engine_off; a los 5 min se duerme igual aunque MQTT nunca haya levantado.
#define PARK_GRACE_MS     60000UL
#define PARK_FORCE_MS     300000UL

// Cada cuanto se anuncia por serial que se esta esperando para parquear. Sin
// esto, "esperando MQTT" y "colgado" se ven identicos desde afuera.
#define PARK_LOG_MS       10000UL

// Espera maxima a que el host abra el puerto USB CDC antes de imprimir. Con
// ARDUINO_USB_CDC_ON_BOOT=1, Serial es el USB nativo del S3 y el host tarda
// 1-2 s en enumerarlo tras un reset: sin esta espera se pierden las primeras
// lineas del arranque, justo las del guardian de bateria.
#define SERIAL_CDC_WAIT_MS 3000UL

// ---- BANCO DE PRUEBAS (todo en 0 para produccion) ----
#define TEST_BAT_OFFSET_V 0.00f   // sube TODOS los umbrales de bateria
#define TEST_FORCE_PARKED 0       // 1 = se comporta apagado aunque haya VBUS
#define TEST_PULSE_S      0UL     // >0 = acorta el pulso para ver varios ciclos

// 1 = NO se duerme nunca, ni por parqueo ni por el guardian de arranque.
//
// Depurar el Nivel 2 en el escritorio es imposible sin esto: cada deep sleep
// hace desaparecer el dispositivo USB, el monitor serial muere y no vuelve, y
// la placa se escapa a dormir a los 60 s del arranque. Toca BOOT+RESET para
// recuperarla. Con esto en 1 el equipo se queda despierto y observable.
//
// OJO: en 1 no se prueba nada del Nivel 2. Sirve para depurar ignicion,
// bateria, LTE y MQTT con el serial vivo. Volver a 0 para probar el parqueo.
#define TEST_DISABLE_SLEEP 0

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
// RTC_DATA_ATTR: que sobreviva al deep sleep, para que el forense "boot" vs
// "mqtt_reconnected" siga significando algo despues de una noche de parqueo.
static RTC_DATA_ATTR bool bootStatusPublished = false;

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

// RTC_DATA_ATTR: el parqueo forzado (MQTT caido) se lleva el evento pendiente a
// dormir. Sin persistirlo, el engine_off se perdia para siempre; asi sale en el
// siguiente pulso de parqueo, aunque llegue tarde.
static RTC_DATA_ATTR PendingEvent pendingEvent = EV_NONE;

// ---------- Ultimo punto valido ----------
// Se cachea para poder publicar el evento de apagado y los pulsos de parqueo
// sin depender de que el GNSS tenga fix fresco en ese momento.
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

// RTC_DATA_ATTR: sin esto, el deep sleep borraba la cache y el pulso de
// parqueo publicaba "aun no hay posicion valida" toda la noche. GpsPoint es
// POD (floats, bools y un char[24]), cabe sin problema en la RTC slow memory.
//
// VALIDADO el 2026-08-20: tras un deep sleep y un despertar por ext0, el
// engine_on salio con el mismo timestamp del engine_off anterior. La cache
// cruzo el sueno intacta.
//
// PENDIENTE (cosmetico): el engine_on no deberia republicar una posicion de
// hace horas. Conviene un limite de antiguedad para ese evento; el engine_off
// si la quiere siempre, porque es el "aqui quedo parqueado".
static RTC_DATA_ATTR GpsPoint lastValidPoint = {};
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
// altitud invalidas, y un HDOP de 0.87 cuando sus vecinas estaban en 0.65; a
// las 03:58 UTC aparecio una tercera, que el subscriber reporto como
// "timestamp no confiable". Ahi la lat/lon salio bien de casualidad; si el
// parseo se corre un campo, publicariamos una coordenada basura y Traccar
// dibuja el carro en otro pais. La fecha es el canario mas confiable del
// parseo, y con cadencia de 5 s descartar la trama completa no cuesta nada.
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

// CAUSA RAIZ del bug que se persiguio toda la sesion del 2026-08-20.
//
// Las primeras conversiones despues de configurar la unidad de ADC son basura.
// Evidencia directa del banco, con el log ya completo:
//
//   [PM] boot#1 wake=0 bat=0.84 vbus=0 ...      <- lectura del guardian
//   [IGN] boot pin=2.700V vbus=5.23V            <- milisegundos despues
//   [BAT] boot 3.68V                            <- y la celda estaba en 4.19 V
//
// Los mismos pines, la misma unidad de ADC, un instante de diferencia: el
// guardian vio 0.84 V de celda y VBUS ausente cuando habia 4.19 V y 5.23 V.
// analogSetPinAttenuation() reconfigura el ADC y las primeras muestras salen
// antes de que la calibracion interna y el sample-and-hold se asienten;
// adcSetup() volvia de inmediato y pmBootGuard() leia acto seguido.
//
// Con el codigo original de la rama, la cuenta del guardian era:
//   !vbus && v < recoverV()  ->  !false && 0.84 < 3.80  ->  DORMIR
// o sea que el equipo se dormia en el arranque con la celda llena. Ese era el
// "pasa a deepsleep desde el inicio", y no tenia nada que ver con los umbrales.
//
// Por eso aqui se descartan conversiones a proposito antes de dar el ADC por
// bueno. Cuesta 40 ms una sola vez por arranque.
static void adcSetup() {
  analogReadResolution(12);
  // 11 dB para cubrir el rango util del divisor (0-3.1 V aprox).
  analogSetPinAttenuation(SENSE_PIN, ADC_11db);
  analogSetPinAttenuation(BAT_ADC_PIN, ADC_11db);

  for (uint8_t i = 0; i < ADC_WARMUP_READS; i++) {
    (void)analogReadMilliVolts(SENSE_PIN);
    (void)analogReadMilliVolts(BAT_ADC_PIN);
    delay(ADC_WARMUP_STEP_MS);
  }
}

// La lectura describe una celda de verdad, o el divisor no esta dando nada?
static bool batteryReadingPlausible(float v) {
  return v >= BAT_PLAUSIBLE_MIN_V;
}

static bool isIgnitionOff() {
#if TEST_FORCE_PARKED
  return true;  // banco: finge carro apagado aunque haya VBUS
#elif IGNITION_SENSE_ENABLED
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
#if TEST_FORCE_PARKED
  // Banco: estado fijo OFF y sin eventos. Sin esto, el pin veria el VBUS del
  // banco y publicaria un engine_on espurio en cada arranque de la prueba.
  ignState = IGN_OFF;
  return;
#endif
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
  // cada 5 s durante un minuto entero con velocidad 0.00. El servidor descarto
  // todas esas tramas con "Skip: no_move", asi que era gasto de radio puro.
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
  uint32_t t0 = millis();
  while (!modem.testAT(1000)) {
    // Este bucle alimenta el WDT, asi que no se reinicia solo: si el modem no
    // responde, el firmware se queda aqui. Al menos que se vea en el log cuanto
    // lleva esperando, para distinguirlo de un cuelgue mudo.
    SerialMon.printf("AT fail, retrying... (%lus)\n",
                     (unsigned long)((millis() - t0) / 1000));
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
//
// DEFECTO CONOCIDO, ajeno a esta rama: el PRIMER intento tras encender el modem
// falla de forma reproducible con state=-4 y un "### Closed: 0" de TinyGSM, y el
// segundo conecta sin problema (banco 2026-08-20). Es el socket zombi que ya
// esta registrado como tarea aparte. El backoff lo absorbe, pero cuesta unos
// segundos en cada arranque y en cada pulso de parqueo, asi que vale la pena
// resolverlo: en el Nivel 2 ese reintento se paga con bateria.
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
  //
  // Nota operativa: como el clientId es fijo, al reflashear el broker expulsa
  // la sesion anterior y publica su will. Por eso en los logs del servidor se
  // ve un "offline" seguido de un "online" unos 200 ms despues; es la toma de
  // relevo, no una caida de enlace.
  //
  // Y al contrario: un "offline" SIN "online" detras significa que la sesion
  // vieja murio y la nueva nunca llego a conectar. Eso paso el 2026-08-20 y es
  // la firma de que el firmware arranco pero se quedo sin MQTT.
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
    // "boot", no hubo ni una reconexion MQTT desde el arranque; si dice
    // "mqtt_reconnected", hubo caidas de enlace pero el firmware sobrevivio.
    // bootStatusPublished vive en RTC_DATA_ATTR precisamente para que este
    // forense no se autodestruya con los despertares del Nivel 2.
    // Verificado en la noche del 2026-08-17 al 18: amanecio en "boot" tras
    // 8 h 22 min y 25 keepalives sin perder uno. Y el 2026-08-20 sirvio para
    // probar que un despertar por ext0 salio de deep sleep y no de un reset:
    // publico "mqtt_reconnected", o sea que la RTC memory sobrevivio.
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
  SerialMon.printf("[BAT] %s V\n", buf);
}

// ===================== NIVEL 2 + CORTE POR BAJO VOLTAJE =====================
// Estado que sobrevive al deep sleep (RTC slow memory).
static RTC_DATA_ATTR uint32_t rtcBootCount    = 0;
static RTC_DATA_ATTR bool     rtcModemAlive   = false;  // modem encendido y sin +CPOF
static RTC_DATA_ATTR bool     rtcInCutoff     = false;
static RTC_DATA_ATTR uint32_t rtcSleptSeconds = 0;
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
  // DTR flotando en los repasos de parqueo de 30 s y el modem despertaria solo.
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

// DEFECTO CORREGIDO (2026-08-20): era un while infinito alimentando el WDT. Si
// el GNSS no encendia, el firmware se quedaba colgado para siempre sin que el
// watchdog lo rescatara. Ahora son intentos acotados y se sigue sin GNSS: el
// tracker todavia sirve para reportar bateria, ignicion y la ultima posicion.
static bool pmGnssOn() {
  for (uint8_t i = 0; i < 20; i++) {
    if (modem.enableGPS(MODEM_GPS_ENABLE_GPIO, MODEM_GPS_ENABLE_LEVEL)) {
      modem.setGPSBaud(115200);

      // Modo GNSS multi-constelacion: 15 = GPS + GLONASS + GALILEO + BDS.
      // AT+CGNSSMODE solo es aceptado por el modem cuando el GNSS ya esta
      // encendido (verificado en banco 2026-08-16, firmware
      // SIM767XM5_B05V01_241206), por eso va DESPUES de enableGPS(). El valor
      // persiste en la NVRAM del modem (sobrevive reflashes del ESP32 y
      // apagados), pero se fija en cada boot para que el comportamiento sea
      // deterministico.
      if (modem.setGPSMode(15)) {
        SerialMon.println("GNSS mode 15 (GPS+GLONASS+GALILEO+BDS)");
      } else {
        SerialMon.println("GNSS mode 15 FAIL (no critico, sigue en modo actual)");
      }
      return true;
    }
    SerialMon.print(".");
    watchdogFeed();
    delay(500);
  }

  SerialMon.println("\n[GNSS] no encendio tras 20 intentos -> se sigue sin GNSS");
  return false;
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
                   (unsigned long)rtcBootCount, (int)esp_sleep_get_wakeup_cause(),
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

  SerialMon.println("[PM] parqueado -> deep sleep");
  pmDeepSleep(true, PARKED_POLL_S);
}

static void pmParkedTick() {
  rtcSleptSeconds += PARKED_POLL_S;
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
      // Nivel 1. ignition=0 en el CSV hace que el subscriber lo deje pasar.
      publishPoint(lastValidPoint, EVENT_NONE);
      delay(SETTLE_MS);
      mqtt.loop();
      pmDisconnectClean();
    } else {
      SerialMon.println("[PM] pulso sin MQTT -> se reintenta en el siguiente");
    }
    pmModemSleep();
    rtcSleptSeconds = 0;
  }

  pmDeepSleep(true, PARKED_POLL_S);
}

void setup() {
  SerialMon.begin(115200);

  // Con ARDUINO_USB_CDC_ON_BOOT=1, Serial es el USB nativo del S3: tras un
  // reset el host tarda 1-2 s en enumerar el puerto y todo lo que se imprima
  // antes se pierde. El 2026-08-20 eso oculto la linea [PM] boot#, que es
  // precisamente la que explica como decidio el guardian de bateria. El
  // operador && corta la espera si nadie esta escuchando, asi que en el carro
  // esto no cuesta nada.
  uint32_t serialWaitStart = millis();
  while (!SerialMon && (millis() - serialWaitStart) < SERIAL_CDC_WAIT_MS) {
    delay(10);
  }
  delay(300);

  watchdogSetup();

#if TEST_DISABLE_SLEEP
  SerialMon.println("[PM] *** TEST_DISABLE_SLEEP=1: no se dormira nunca (modo banco) ***");
#endif

  // OBLIGATORIO antes de pmBootGuard(): configura la atenuacion Y calienta el
  // ADC descartando las primeras conversiones. Sin esto el guardian decide
  // sobre ruido y duerme el equipo con la celda llena (banco 2026-08-20).
  adcSetup();

  pmBootGuard();            // si la celda esta bajo el piso, duerme aqui mismo

  // Arranca el reloj del ralenti desde el boot. Sin esto, un vehiculo encendido
  // que nunca supera MOVING_SPEED_KMH se queda para siempre en cadencia rapida.
  lastMovementMs = millis();

  // Lectura informativa de arranque. El estado real se resuelve en el loop con
  // su antirrebote: hasta entonces IGN_UNKNOWN se comporta como encendido.
  float bootPinV = readPinVolts();
  SerialMon.printf("[IGN] boot pin=%.3fV vbus=%.2fV (sense %s)\n",
                   bootPinV, bootPinV * DIVIDER_FACTOR,
                   IGNITION_SENSE_ENABLED ? "activo" : "desactivado");
  SerialMon.printf("[BAT] boot %.2fV\n", readBatteryVolts());

  // Solo abre el UART. No enciende nada: el repaso de parqueo debe poder
  // decidir y volver a dormir sin tocar el modem.
  SerialAT.begin(MODEM_BAUDRATE, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);

  // Despertar por temporizador sin ignicion = seguir parqueado.
  // OJO: ignState no sobrevive al deep sleep (arranca en IGN_UNKNOWN), asi que
  // la decision se toma del pin directamente; isIgnitionOff() solo esta en la
  // condicion para cubrir el banco (TEST_FORCE_PARKED), donde el pin esta alto
  // por el VBUS de la fuente.
  //
  // El umbral es PIN_ON_V y no PIN_OFF_V a proposito: con el corte anterior, un
  // pin en zona media (0.6-2.5 V, divisor sucio o cap descargandose) disparaba
  // un arranque completo con modem + LTE + MQTT + GNSS cada 30 segundos, que es
  // la forma mas rapida que existe de vaciar la 18650. Solo un pin francamente
  // alto significa ignicion encendida.
#if !TEST_DISABLE_SLEEP
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER &&
      (isIgnitionOff() || bootPinV < PIN_ON_V)) {
    pmParkedTick();         // NO retorna
  }
#endif

  // PWRKEY solo si el modem estaba apagado de verdad: pulsarlo con el modem
  // encendido lo APAGA.
  pmModemResume();

  // 1) LTE up
  ensureLTE();

  // 2) MQTT up (primer intento; el loop se encarga de reintentar).
  //    El status "boot" lo publica tryConnectMQTT() al conectar.
  //    Ver la nota de tryConnectMQTT(): este primer intento falla de forma
  //    reproducible con state=-4 y conecta en el reintento del loop.
  tryConnectMQTT();

  // 3) GPS on
  SerialMon.println("Enabling GPS...");
  if (pmGnssOn()) {
    SerialMon.println("GPS Enabled");
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

  // Corte por bajo voltaje: vigila en cada vuelta (muestrea cada 10 s).
  if (pmCheckCutoff()) pmEnterCutoff(readBatteryVolts());  // NO retorna

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

  // Nivel 2: dormir cuando el carro esta apagado.
  //
  // DEFECTO CORREGIDO (2026-08-20): la condicion exigia pendingEvent == EV_NONE,
  // pero serviceEvents() solo limpia el pendiente cuando MQTT esta conectado.
  // Con MQTT caido el evento nunca se limpiaba, la valvula de escape de 5 min
  // era inalcanzable y el equipo se quedaba despierto vaciando la celda:
  // exactamente el escenario que el Nivel 2 venia a evitar. Ahora el escape es
  // por tiempo puro y pendingEvent vive en RTC, asi que el engine_off no se
  // pierde: sale en el siguiente pulso de parqueo.
  bool graced      = (millis() > PARK_GRACE_MS);
  bool eventsDone  = (mqtt.connected() && pendingEvent == EV_NONE);
  bool giveUpOnNet = (millis() > PARK_FORCE_MS);

  // Anuncio periodico de por que todavia no se parquea. Sin esta linea, un
  // equipo esperando MQTT y un equipo colgado se ven identicos desde afuera:
  // ni publica ni imprime. El 2026-08-20 se perdio media hora deduciendo por
  // consumo (20-40 mA) y por un LWT lo que esta linea habria dicho de frente.
  static uint32_t lastParkLogMs = 0;
  if (isIgnitionOff() && (millis() - lastParkLogMs) > PARK_LOG_MS) {
    lastParkLogMs = millis();
    SerialMon.printf("[PM] apagado t=%lus mqtt=%d pend=%u gracia=%d forzado_en=%lus\n",
                     (unsigned long)(millis() / 1000), (int)mqtt.connected(),
                     (unsigned)pendingEvent, (int)graced,
                     (unsigned long)(PARK_FORCE_MS / 1000));
  }

#if !TEST_DISABLE_SLEEP
  if (isIgnitionOff() && graced && (eventsDone || giveUpOnNet)) {
    if (!eventsDone) {
      SerialMon.println("[PM] parqueo forzado: MQTT no levanto, el evento queda en RTC");
    }
    pmEnterParked();        // NO retorna
  }
#endif

  delay(10);
}
