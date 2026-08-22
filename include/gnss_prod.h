#pragma once
//
// gnss_prod.h — motor GNSS de produccion
// Proyecto GPS Tracker Logan · rama feat/low-power-nivel2 · 2026-08-22
//
// ESTE ARCHIVO NO ES INDEPENDIENTE. Es un pedazo de main.cpp que vive aparte
// para que main.cpp no siga creciendo. Se incluye UNA sola vez, desde main.cpp,
// justo despues de la definicion de watchdogFeed(). En ese punto ya existen y
// por eso aqui se usan sin declararlos:
//
//   modem, SerialMon, watchdogFeed(),
//   MODEM_GPS_ENABLE_GPIO, MODEM_GPS_ENABLE_LEVEL
//
// A proposito NO usa mqtt ni lastValidPoint, aunque los necesite logicamente.
// Motivo: en main.cpp esos dos se declaran despues de tryConnectMQTT(), y
// tryConnectMQTT() necesita la huella de build de aqui. Si el header dependiera
// de ellos no habria ningun punto valido donde incluirlo. Todo lo que viene de
// ese lado entra por parametro y la publicacion MQTT la hace main.cpp.
//
// ---------------------------------------------------------------------------
// SEXTA REGLA DEL PROYECTO
//
//   Que una libreria devuelva false no significa que la operacion fallo.
//
// TinyGSM::enableGPS() manda AT+CGNSSPWR=1 y espera hasta 30 segundos el URC
// "+CGNSSPWR: READY!". Ese URC NO EXISTE en el firmware de este modem
// (SIM767XM5_B05V01_241206). Comprobado en tres corridas del laboratorio GNSS:
// v1.1, v1.2 y v1.4. El modem contesta OK en unos 70 ms y el motor queda
// operativo a los ~2 s, pero enableGPS() devuelve false porque nunca ve su
// URC. El GNSS quedo encendido y respondiendo AT+CGNSSINFO todo el tiempo.
//
// El costo real de creerle a ese false: pmGnssOn() reintentaba 20 veces, y
// 20 x (30 s + 0,5 s) son MAS DE DIEZ MINUTOS de setup() bloqueado en cada
// arranque. Diez minutos sin muestrear la ignicion, sin reintentar MQTT y sin
// publicar una sola posicion. El watchdog de 120 s no salta porque el bucle
// alimentaba watchdogFeed() en cada vuelta: el firmware se colgaba "bien
// portado". Y al final imprimia "no encendio tras 20 intentos" sobre un GNSS
// que llevaba diez minutos prendido.
//
// ---------------------------------------------------------------------------
// SEPTIMA REGLA DEL PROYECTO
//
//   Una lectura con buenos numeros no es una lectura nueva. Solo la marca de
//   tiempo delata la trama vieja.
//
// Medido en el laboratorio v1.4, corrida T10: despues de AT+CGNSSWAKEUP el
// primer +CGNSSINFO devolvio un fix con 27 satelites y HDOP 1,4 y marca de
// tiempo 220826/165136.000 — exactamente la del fix tomado 150 segundos ANTES,
// antes de dormir el motor. El fix real llego 45 segundos despues, con marca
// 220826/165452.000.
//
// Lo peligroso es que la trama vieja se veia MEJOR que las reales: 27 sats
// contra 26, HDOP 1,4 contra 1,6. Ningun filtro de calidad la atrapa. Ni
// isGpsPositionValid(), ni el rango de velocidad, ni el canario de fecha.
// Sin guardia, T10 habria reportado un TTFF de 0 segundos y nos habriamos
// felicitado por un fix que era del pasado.
//

// ---------------------------------------------------------------------------
// F19 — parametros del encendido
// ---------------------------------------------------------------------------

// AT+CGNSSPWR=1 contesta OK en ~70 ms. Diez segundos es holgura de sobra.
#define GNSS_PWR_TIMEOUT_MS 10000UL

// El motor queda operativo a los ~2 s. No es una espera de fix, es el
// asentamiento del chip antes de la primera consulta.
#define GNSS_SETTLE_MS 2000UL

// Dos intentos, no veinte. Si el modem no contesta OK dos veces seguidas a un
// comando que tarda 70 ms, el problema no se arregla insistiendo.
#define GNSS_PWR_ATTEMPTS 2

// 1 = reescribir CGNSSMODE/CGNSSIPR en cada arranque.
//
// Se deja en 0 porque ambos son SAVE-persistentes en la NVRAM del modem y el
// laboratorio del 2026-08-22 encontro el modo en 15 en TODAS las corridas,
// incluidas las que venian de AT+CPOF y de AT+CRESET. El manual le da hasta
// 10 s a cada comando, asi que reescribirlos es pagar sin comprar nada.
// Se consulta y queda en el log.
#define GNSS_FORCE_MODE 0

// ---------------------------------------------------------------------------
// F22 — parametros de la guardia de trama rancia
// ---------------------------------------------------------------------------

// Techo de reporte. El contador sube siempre; el log se limita para que un
// GNSS pegado no inunde el serial.
#define GNSS_STALE_LOG_MS 60000UL

// ---------------------------------------------------------------------------
// F16 — huella de build
// ---------------------------------------------------------------------------
//
// "Commiteado" no es "flasheado". Esta sesion perdio tiempo dos veces por
// analizar el comportamiento de un binario viejo. __DATE__ y __TIME__ los pone
// el compilador, asi que no hay forma de que esto mienta.

#define FW_NAME "tracker"
#define FW_VERSION "2.1"
static const char FW_BUILD[] = __DATE__ " " __TIME__;

static const char TOPIC_FW[] = "tracker/Lilygo/sys/fw";
static const char TOPIC_GNSS[] = "tracker/Lilygo/sys/gnss";

// ---------------------------------------------------------------------------
// Estado
// ---------------------------------------------------------------------------

// Tramas rancias descartadas. En RTC: si el defecto aparece a las 3 de la
// manana durante un ciclo de parqueo, el numero tiene que seguir ahi despues
// del deep sleep. Es el unico testigo que vamos a tener.
static RTC_DATA_ATTR uint16_t rtcGnssStale = 0;

// Cronometro del TTFF real. NO va en RTC: mide un encendido concreto, no una
// acumulacion, y despues de un deep sleep un millis() viejo no significa nada.
static uint32_t gnssOnAtMs = 0;
static bool gnssTtffPending = false;

// ---------------------------------------------------------------------------
// F19 — encendido del motor GNSS
// ---------------------------------------------------------------------------
//
// Reemplaza a modem.enableGPS(). Manda la misma secuencia AT que la libreria
// (CGDRT para habilitar el GPIO de la antena activa, CGSETV para su nivel,
// CGNSSPWR=1 para el power) pero NO espera el URC READY que este firmware no
// emite. Devuelve true cuando el modem confirmo el power con OK.
static bool gnssPwrOn() {
  // Alimentacion de la antena activa. Sin esto el chip enciende pero mira al
  // suelo: es lo que hace enableGPS() antes del power y hay que conservarlo.
  modem.sendAT("+CGDRT=", MODEM_GPS_ENABLE_GPIO, ",1");
  modem.waitResponse(2000);
  modem.sendAT("+CGSETV=", MODEM_GPS_ENABLE_GPIO, ",", MODEM_GPS_ENABLE_LEVEL);
  modem.waitResponse(2000);

  bool ok = false;
  for (uint8_t i = 0; i < GNSS_PWR_ATTEMPTS && !ok; i++) {
    modem.sendAT("+CGNSSPWR=1");
    ok = (modem.waitResponse(GNSS_PWR_TIMEOUT_MS) == 1);
    if (!ok) {
      SerialMon.printf("[GNSS] AT+CGNSSPWR=1 sin OK (intento %u de %u)\n",
                       (unsigned)(i + 1), (unsigned)GNSS_PWR_ATTEMPTS);
      watchdogFeed();
      delay(500);
    }
  }

  if (!ok) {
    SerialMon.println("[GNSS] el modem no confirmo el power -> se sigue sin GNSS");
    return false;
  }

  // Aqui es donde el codigo viejo perdia diez minutos esperando
  // "+CGNSSPWR: READY!". No se espera. Solo el asentamiento del chip.
  delay(GNSS_SETTLE_MS);

  gnssOnAtMs = millis();
  gnssTtffPending = true;

#if GNSS_FORCE_MODE
  modem.setGPSBaud(115200);
  if (modem.setGPSMode(15)) {
    SerialMon.println("[GNSS] modo 15 escrito (GPS+GLONASS+GALILEO+BDS)");
  } else {
    SerialMon.println("[GNSS] modo 15 FAIL (no critico, sigue en el modo guardado)");
  }
#else
  // Solo consulta, para que el modo quede en el log de cada arranque.
  // TINY_GSM_DEBUG ya ecoa la respuesta al serial.
  modem.sendAT("+CGNSSMODE?");
  modem.waitResponse(2000);
#endif

  SerialMon.printf("[GNSS] motor encendido en ~%lu ms (antes: hasta 610000 ms)\n",
                   (unsigned long)GNSS_SETTLE_MS);
  return true;
}

// ---------------------------------------------------------------------------
// F22 — guardia de trama rancia
// ---------------------------------------------------------------------------
//
// Devuelve true si esta lectura trae la MISMA marca de tiempo que el ultimo
// punto valido, es decir, si el modem nos esta reciclando una solucion vieja.
// Cuando devuelve true hay que descartar la lectura entera.
//
// La referencia entra por parametro (lastValidPoint vive en RTC_DATA_ATTR en
// main.cpp, asi que sobrevive al deep sleep sin variable nueva).
static bool gnssFrameIsStale(const char *ts, bool lastValid, const char *lastTs) {
  if (!lastValid) return false;
  if (strcmp(ts, lastTs) != 0) return false;

  rtcGnssStale++;

  static uint32_t lastStaleLogMs = 0;
  uint32_t nowMs = millis();
  if (lastStaleLogMs == 0 || (nowMs - lastStaleLogMs) > GNSS_STALE_LOG_MS) {
    lastStaleLogMs = nowMs;
    SerialMon.printf("[GPS] trama rancia: marca %s repetida -> descartada (total=%u)\n",
                     ts, (unsigned)rtcGnssStale);
  }
  return true;
}

// ---------------------------------------------------------------------------
// TTFF real de produccion
// ---------------------------------------------------------------------------
//
// Se llama con el primer fix FRESCO despues de cada encendido del motor. Hasta
// hoy todos los TTFF que conocemos son de banco; este es el primero que sale
// del tracker haciendo su trabajo.
static void gnssNoteFirstFix(int sats, float hdop) {
  if (!gnssTtffPending) return;
  gnssTtffPending = false;
  SerialMon.printf("[GNSS] TTFF real %lus desde el encendido (sats=%d hdop=%.2f)\n",
                   (unsigned long)((millis() - gnssOnAtMs) / 1000), sats, hdop);
}

// ---------------------------------------------------------------------------
// F20 — diagnostico de unidad de velocidad
// ---------------------------------------------------------------------------
//
// SOLO INSTRUMENTACION. No se toca el CSV a proposito.
//
// El protocolo OsmAnd de Traccar espera NUDOS. Si el modem entrega nudos, el
// pipeline actual acierta por accidente y cambiar las unidades lo romperia. Si
// entrega km/h, llevamos meses reportando velocidades infladas 1,852 veces.
// No hay dato que lo decida, asi que se imprimen las dos lecturas y se cierra
// contra el velocimetro en la proxima prueba de manejo.
static void gnssLogSpeedUnits(float speed) {
  if (speed <= 0.5f) return;
  SerialMon.printf("[GPS] velocidad cruda=%.2f (si es km/h -> %.1f | si es nudos -> %.1f km/h)\n",
                   speed, speed, speed * 1.852f);
}

// ---------------------------------------------------------------------------
// F16 — cadena de la huella
// ---------------------------------------------------------------------------
//
// Arma el texto; publicar es cosa de main.cpp, que es quien tiene el cliente
// MQTT a la vista.
static void gnssFwString(char *buf, size_t n) {
  snprintf(buf, n, "%s %s,%s", FW_NAME, FW_VERSION, FW_BUILD);
}
