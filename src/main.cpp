// ============================================================================
// GPS Tracker Logan - firmware principal (env: tracker)
//
// Telemetria GNSS por LTE hacia MQTT + sense de ignicion por ADC en GPIO9.
//
// El estado de la ignicion gobierna la cadencia de publicacion:
//   IGN_ON  en marcha  -> cada GPS_PERIOD_MS        (5 s)
//   IGN_ON  en ralenti -> cada IDLE_PERIOD_MS       (30 s)
//   IGN_OFF parqueado  -> engine_off y luego deep sleep (Nivel 2): repaso de
//                         ignicion cada 120 s y un pulso de bateria/posicion al dia
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
// CORRECCION DE ESA NOTA (2026-08-22): lo de AT+CAGPS quedo mal escrito. El
// comando NO da ERROR: contesta OK. Lo que pasa es que el laboratorio GNSS lo
// midio y empeoro el TTFF de 22 s a 192 s, y ademas AT+CAGPS ni siquiera es el
// comando documentado para esta familia (el manual V1.02 lista AT+CGNSSAGPS en
// el capitulo 21.2.23, y advierte que puede contestar OK habiendo fallado, con
// el motivo real en un URC aparte). A-GNSS queda PENDIENTE DE PRUEBA con el
// comando correcto, no descartado.
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
// por ext0 si la ignicion sube y, como red de seguridad, cada 120 s por
// temporizador. Una vez al dia (PARKED_PULSE_S) despierta el modem, publica
// bateria + posicion y vuelve a dormir. El corte por bajo voltaje
// (BAT_CUTOFF_V, con histeresis BAT_RECOVER_V) apaga TODO con AT+CPOF y solo
// rearma con la celda recuperada o con VBUS presente. El guardian de arranque
// bloquea cualquier boot sin VBUS por debajo del corte, sin encender el
// modem: eso mata el bucle de brownout del 2026-08-18 (10 reconexiones en
// 108 s, muerte a 2.37 V).
//
// MEDIDO (noche del 2026-08-19 al 20, 5 h 35 min parqueado con TEST_PULSE_S de
// una hora): 5 de 5 pulsos entregados, cache de posicion y contadores RTC
// intactos toda la noche, y la 18650 bajo de 4.15 V a 4.10 V en 4 h 28 min,
// unos 11.2 mV/h contra los 38 mV/h del Nivel 1.
//
// OJO AL LEER ESA CIFRA: 4.19-4.10 V es la zona mas plana de la curva de la
// 18650, asi que en mV/h el Nivel 2 se ve mejor de lo que es. Traducido a
// capacidad son ~1.25 %/h contra ~2.87 %/h del Nivel 1: una mejora real de
// 2.3x, no de 3.4x. La medicion honesta pide dejarla correr hasta 3.80 V.
//
// Y esa noche todavia cargaba el desperdicio de la espera del CDC USB (3.3 s
// de CPU a plena potencia en cada repaso de 30 s, un 10 % de ciclo util), que
// ya esta corregido. Ver el comentario de SERIAL_CDC_WAIT_MS.
//
// ENTREGADO A MQTT NO ES ENTREGADO A TRACCAR (leccion del 2026-08-20, por la
// mañana): esos 5 pulsos si estaban en el broker, pero el subscriber los tiro
// todos con "Skip: no_move_0.0m" y la noche entera quedo invisible en el mapa.
// El CSV decia ignition=1 porque ignState no sobrevivia al deep sleep, y sin
// ignition=0 no hay bypass del filtro de movimiento. Una prueba de campo no
// termina en mosquitto_sub: termina en el log del contenedor. Ver ignState y
// pmParkedTick().
//
// OJO CON EL RETRASO DE DETECCION: los 120 s del repaso NO son la latencia de
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
//
// CUARTA REGLA (la mañana siguiente): en parqueo, cada milisegundo despierto se
// paga con celda. Todo lo que se agregue al camino de arranque corre cientos de
// veces al dia, asi que ninguna espera va antes de saber POR QUE despertamos.
// La observabilidad se agrega para el arranque en frio, no para el repaso.
//
// QUINTA REGLA (media hora despues de la cuarta): todo estado del que dependa
// una publicacion tiene que vivir en RTC_DATA_ATTR o recalcularse en el
// despertar. Un deep sleep no es una pausa: es un reset con memoria selectiva,
// y las variables normales vuelven a su valor inicial. Ver ignState.
//
// SEXTA REGLA (2026-08-22, laboratorio GNSS): que una libreria devuelva false
// no significa que la operacion haya fallado. modem.enableGPS() reportaba
// fracaso durante DIEZ MINUTOS en cada arranque mientras el GNSS ya estaba
// encendido y contestando: se quedaba esperando un URC que este firmware no
// emite nunca. Antes de reintentar veinte veces hay que comprobar en el banco
// que la operacion de verdad fracaso. Ver pmGnssOn().
//
// SEPTIMA REGLA (misma sesion): una lectura con buenos numeros no es una
// lectura NUEVA. El modem puede devolver la ultima solucion GNSS guardada, con
// mas satelites y mejor HDOP que la real, y ningun filtro de calidad la
// distingue. Lo unico que delata a una trama vieja es que su marca de tiempo no
// avanzo. Ver readGpsPoint().
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

// ---------- Huella de build (F16) ----------
// Saber QUE esta corriendo en la placa sin tener que adivinar. El 2026-08-22 se
// perdio tiempo justamente por eso: "commiteado" no es "flasheado", y con cinco
// versiones del laboratorio en una mañana era imposible saber cual estaba
// arriba. Se publica RETENIDO, asi que basta suscribirse al topic para saber
// que firmware corre, aunque el equipo lleve horas dormido.
#define FW_NAME    "tracker"
#define FW_VERSION "2.1"
static const char FW_BUILD[] = __DATE__ " " __TIME__;

// ---------- Identidad del dispositivo ----------
// APN, APN_USER, APN_PASS, MQTT_HOST, MQTT_PORT, MQTT_USER y MQTT_PASS
// se definen en include/secrets.h

const char DEVICE_ID[] = "Lilygo";
const char TOPIC_TELEMETRY[] = "tracker/Lilygo/telemetria";

// Topics de sistema (bajo volumen, QoS 1 / retained donde aplica)
const char TOPIC_LWT[]    = "tracker/Lilygo/sys/lwt";
const char TOPIC_STATUS[] = "tracker/Lilygo/sys/status";
const char TOPIC_FW[]     = "tracker/Lilygo/sys/fw";

// Forense del GNSS: contador de tramas rancias descartadas. Retenido y de muy
// bajo volumen (solo se publica cuando el contador se mueve, y como maximo una
// vez por minuto). Si este topic crece en campo, el defecto de la trama vieja
// esta pasando de verdad y no solo en el banco.
const char TOPIC_GNSS[]   = "tracker/Lilygo/sys/gnss";

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
//
// Y CONFIRMADO POR LA VIA DOLOROSA la noche del 19 al 20: los 5 pulsos de
// parqueo salieron con ignition=1 (ignState no sobrevivia al sueño) y el
// subscriber los tiro con "Skip: no_move_0.0m", los cinco. Cero puntos en
// Traccar. Ese campo no es decorativo: es la unica llave del bypass.
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

// Cada cuanto despierta el ESP32 mientras el carro esta parqueado.
//
// SUBIDO DE 30 A 120 S (2026-08-20) con datos en mano. Cada repaso es un
// arranque completo del ESP32, y en la noche del 19 al 20 esos despertares
// resultaron ser el consumidor DOMINANTE del parqueo: mas que el modem dormido
// por DTR, que ya estaba en el orden de miliamperios. Con la espera del CDC
// USB fuera del camino, un repaso cuesta ~0.5 s, asi que el ciclo util pasa de
// ~10 % (3.3 s de cada 33.5 s) a ~0.4 %.
//
// Esto NO afecta la latencia de deteccion de la ignicion: de eso se encarga
// ext0, que es una interrupcion de hardware y es instantanea. Lo unico que se
// espacia es la vigilancia del voltaje de la celda, y con BAT_CUTOFF_N = 5 el
// corte pasa a exigir 10 minutos sostenidos bajo el piso. Para una 18650 que
// tarda horas en moverse 50 mV, eso sigue siendo de sobra.
#define PARKED_POLL_S     120UL
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
//
// SOLO EN ARRANQUE EN FRIO. Y no es un detalle de estilo: cuando esta espera
// corria en todos los arranques, con el USB desconectado !SerialMon nunca se
// hacia falso y el bucle agotaba los 3000 ms completos, con el CPU a plena
// potencia y alimentado por la 18650, en CADA repaso de parqueo. La noche del
// 19 al 20 eso se midio solo: los pulsos pedidos a 60 min salieron a 67 min
// clavados, +11.8 %, porque cada ciclo de 30 s duraba 33.5 s reales. Un 10 %
// de ciclo util regalado por esperar a un puerto que nadie estaba mirando.
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

// ---------- Encendido del motor GNSS ----------
// Techo de respuesta de AT+CGNSSPWR=1. Medido en banco: contesta OK en 70 ms.
// Diez segundos son de sobra y no se parecen en nada a los 30 s que esperaba
// la libreria por un URC inexistente.
#define GNSS_PWR_TIMEOUT_MS 10000UL

// Cuanto tarda el motor en quedar operativo despues del OK. Medido en las tres
// corridas del laboratorio: a los ~2 s ya responde AT+CGNSSINFO.
#define GNSS_SETTLE_MS      2000UL

// Intentos de encendido. Antes eran 20 x 30 s = mas de diez minutos. Ahora dos
// intentos de 10 s como maximo, y si no enciende se sigue sin GNSS igual que
// antes: el tracker todavia sirve para bateria, ignicion y ultima posicion.
#define GNSS_PWR_ATTEMPTS   2

// 1 = reescribe AT+CGNSSMODE y AT+CGNSSIPR en cada arranque.
//
// Se deja en 0 con datos: ambos comandos son SAVE-persistentes en la NVRAM del
// modem, y el laboratorio del 2026-08-22 encontro CGNSSMODE en 15 en TODAS las
// corridas -incluidas las que venian de AT+CPOF y de AT+CRESET-. Reescribirlo
// en cada boot no aporta determinismo y el manual le da hasta 10 s de tiempo de
// respuesta a cada uno. En su lugar se consulta y queda en el log, que con
// TINY_GSM_DEBUG activo se imprime solo.
#define GNSS_FORCE_MODE     0

// Cada cuanto, como maximo, se reporta una trama rancia. Son raras, pero si el
// motor se queda pegado en la solucion vieja sondeamos cada 5 s y no tiene
// sentido llenar el log ni el broker.
#define GNSS_STALE_LOG_MS   60000UL

// ---------- Validación GNSS ----------
// PENDIENTE: verificar la unidad real que entrega AT+CGNSSINFO. La evidencia
// de campo (Traccar coincide con el odómetro del vehículo) apunta a que el
// módem reporta NUDOS y no km/h, por lo que este umbral estaría filtrando en
// realidad a ~333 km/h. No cambiar hasta confirmarlo con más mediciones.
//
// INSTRUMENTADO EL 2026-08-22 (F20): readGpsPoint() ahora imprime la velocidad
// cruda y sus dos interpretaciones posibles cada vez que hay movimiento. Con
// una prueba de manejo y el velocimetro al lado, esto se cierra solo. NO se
// cambia el CSV: el protocolo OsmAnd de Traccar espera nudos, asi que si el
// modem entrega nudos el pipeline actual esta bien por accidente y tocarlo a
// ciegas romperia lo que hoy funciona.
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

// RTC_DATA_ATTR (DEFECTO CORREGIDO 2026-08-20, encontrado en produccion).
//
// Sin esto, cada despertar del Nivel 2 arrancaba en IGN_UNKNOWN, que el
// firmware trata como "encendido" por fail-safe. Ese fail-safe es correcto en
// un arranque en frio y es EXACTAMENTE LO CONTRARIO de lo que se necesita en un
// repaso de parqueo, donde la unica razon de estar despierto es que el carro
// esta apagado. Consecuencia medida: los 5 pulsos de la noche del 19 al 20
// publicaron ignition=1, el subscriber perdio el bypass, y los descarto con
// "Skip: no_move_0.0m" porque la trama era identica a la anterior (misma
// posicion cacheada, mismo timestamp). Cinco pulsos entregados a MQTT, cero
// puntos en Traccar, la noche entera invisible en el mapa.
//
// Prueba por contraste del mismo log: el engine_off de las 01:34:24, con la
// MISMA lat/lon que un "Skip: no_move_4.4m" de nueve segundos antes, paso sin
// problema. El filtro no estaba roto; el firmware le estaba mintiendo.
//
// Guardarlo en RTC hace que el estado resuelto con antirrebote antes de
// parquear (IGN_OFF) siga vigente en cada repaso, que es la verdad del mundo
// fisico: el carro no se enciende solo mientras el ESP32 duerme, y si se
// enciende, para eso esta ext0.
static RTC_DATA_ATTR IgnState ignState = IGN_UNKNOWN;
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

// Causa del ultimo despertar, resuelta una sola vez al entrar a setup(). Se
// consulta antes que nada porque de ella depende si vale la pena esperar el
// puerto USB: en un repaso de parqueo, esa espera es celda tirada a la basura.
static esp_sleep_wakeup_cause_t wakeCause = ESP_SLEEP_WAKEUP_UNDEFINED;

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
// cruzo el sueno intacta. Y la noche del 19 al 20 aguanto 5 h 35 min y unos
// 600 despertares seguidos sin corromperse.
//
// Y desde el 2026-08-22 cumple una SEGUNDA funcion: su campo ts es la
// referencia contra la que readGpsPoint() detecta las tramas rancias. Por eso
// tiene que seguir viviendo en RTC aunque algun dia se quite la cache: si se
// pierde entre suenos, el guardia arranca ciego en cada despertar, que es
// justo el momento en que el modem devuelve la solucion vieja.
//
// PENDIENTE (cosmetico): el engine_on no deberia republicar una posicion de
// hace horas. Conviene un limite de antiguedad para ese evento; el engine_off
// si la quiere siempre, porque es el "aqui quedo parqueado".
static RTC_DATA_ATTR GpsPoint lastValidPoint = {};

// Contador de tramas rancias descartadas. En RTC para que sobreviva a los
// cientos de despertares del parqueo: si el defecto aparece de madrugada, el
// numero tiene que seguir ahi por la mañana.
static RTC_DATA_ATTR uint16_t rtcGnssStale = 0;

// Cronometro del TTFF real en produccion. Se arma cuando pmGnssOn() enciende el
// motor y se desarma con el primer fix fresco. No va en RTC a proposito: mide
// un encendido concreto, no una acumulacion.
static uint32_t gnssOnAtMs = 0;
static bool     gnssTtffPending = false;

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
// CONFIRMADO el mismo dia: con el calentamiento puesto, el mismo boot leyo
// bat=4.18 vbus=1 y no hubo que recurrir a ninguna guarda.
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

// Campo "ignition" del CSV. NO es solo informativo: en el subscriber,
// ignition=0 (o un event != "-") es lo que hace que la trama salte el
// rate-limit y el filtro de movimiento minimo. Si esto sale en 1 con el carro
// parqueado, el punto no llega a Traccar. Ver el comentario de ignState.
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
      LWT_OFFL