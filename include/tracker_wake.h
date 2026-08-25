#pragma once
//
// tracker_wake.h — forense y validacion del despertar
// Proyecto GPS Tracker Logan · rama feat/low-power-nivel2
//
// ESTE ARCHIVO NO ES INDEPENDIENTE. Se incluye UNA sola vez desde main.cpp,
// DESPUES de tracker_pm.h y ANTES de setup(). En ese punto ya existen y por eso
// aqui se usan sin declararlos: mqtt, SerialMon, wakeCause, ignState, IGN_OFF,
// readPinVolts(), watchdogFeed(), los umbrales PIN_ON_V / ON_DEBOUNCE_MS /
// IGN_SAMPLE_MS / PARKED_POLL_S, las banderas TEST_*, rtcBootCount,
// rtcSpuriousExt0 y pmDeepSleep() / pmParkedTick() de tracker_pm.h, y
// fixAgeS() de tracker_telemetry.h.
//
// Se creo aparte por la misma razon por la que existen tracker_pm.h y
// tracker_telemetry.h: main.cpp pesa 45 703 B y el techo practico de escritura
// remota esta en ~43 KB. El 2026-08-22 un intento de reescribirlo completo lo
// dejo truncado a mitad de tryConnectMQTT() y el repo sin compilar. Metiendo la
// logica aqui, main.cpp solo necesita un #include y tres llamadas.
//
// Existe por dos defectos de campo de la madrugada del 2026-08-24 que comparten
// la misma raiz: el firmware le creia a su memoria y al enganche de hardware, y
// nunca le preguntaba al pin.
//

// Topic retenido con la forense de cada arranque que alcanza a tener MQTT: por
// que arranco, con cuanto voltaje en el pin, cuantos ext0 espurios se han
// descartado desde el ultimo reset y que tan vieja esta la posicion en cache.
//
// Es la instrumentacion que va a decidir de donde salen los picos del pin: si
// aparecen con regularidad de reloj, el sospechoso es el modem, que sigue
// registrado en LTE y despierta solo a hablar con la red con rafagas de casi un
// amperio a centimetros de la pista del sense. Si aparecen desordenados y
// coinciden con abrir el carro, es la electricidad del vehiculo.
#define TOPIC_WAKE "tracker/Lilygo/sys/wake"

// Valores de wakeCause que salen en ese topic, para poder leerlo sin el manual
// de ESP-IDF al lado:
//   0 = arranque en frio o reset (boton, brownout, watchdog)
//   2 = ext0, o sea el pin de ignicion
//   4 = temporizador (repaso de parqueo)

// ===========================================================================
// DEFECTO 1 — LA SORDERA. Corregido 2026-08-24.
//
// La condicion que habia en setup() era:
//
//   wakeCause == ESP_SLEEP_WAKEUP_TIMER && (isIgnitionOff() || bootPinV < PIN_ON_V)
//
// ignState vuelve del RTC en IGN_OFF SIEMPRE que venimos de un parqueo, asi que
// isIgnitionOff() era true y el || cortocircuitaba antes de llegar a mirar el
// pin. Resultado: el repaso de PARKED_POLL_S se volvia a dormir aunque el carro
// estuviera andando.
//
// Normalmente ext0 tapaba el hueco, pero ext0 solo se arma si el pin esta BAJO
// en el instante exacto de dormirse (ver pmDeepSleep). Esa madrugada el ultimo
// parqueo cayo justo despues de dos picos en el pin, ext0 no quedo armado, y el
// tracker quedo SORDO: 1 h 32 min de silencio total mientras el usuario manejaba,
// unos 46 repasos que consultaron su propia memoria y se volvieron a dormir.
// Apagar y prender el carro no sirvio. Solo el boton de reset lo revivio, porque
// un arranque en frio no pasa por aqui.
//
// Ahora la decision se toma del PIN, que es exactamente lo que el comentario
// original de main.cpp declaraba y el codigo no hacia. Con esto el repaso de
// parqueo pasa a ser la red de seguridad que siempre debio ser: aunque ext0
// falle, en maximo PARKED_POLL_S el tracker se da cuenta de que el carro esta
// prendido.
// ===========================================================================
static void wakeServiceTimerTick(float bootPinV) {
#if !TEST_DISABLE_SLEEP
  if (wakeCause != ESP_SLEEP_WAKEUP_TIMER) {
    return;
  }

#if TEST_FORCE_PARKED
  // En banco el pin ve el VBUS de la fuente y nunca baja: se fuerza el parqueo.
  bool sigueParqueado = true;
#else
  bool sigueParqueado = (bootPinV < PIN_ON_V);
#endif

  if (sigueParqueado) {
    pmParkedTick();   // NO RETORNA: termina en deep sleep
  }

  SerialMon.printf("[PM] repaso con el pin ARRIBA (%.3fV >= %.2fV) -> el carro esta prendido, sigue el arranque\n",
                   bootPinV, PIN_ON_V);
#else
  (void)bootPinV;
#endif
}

// ===========================================================================
// DEFECTO 2 — DESPERTARES ESPURIOS POR EL PIN. Corregido 2026-08-24.
//
// Esa misma noche, con el carro apagado y quieto desde las 21:39Z, hubo CINCO
// despertares por ext0 (04:20:10, 04:21:11, 05:55:39, 09:03:43 y 09:04:36 Z).
// Cada uno prendio el modem, levanto LTE, conecto MQTT, encendio el GNSS,
// publico la posicion cacheada como engine_off y se volvio a dormir al minuto.
// La celda bajo de 4.11 a 4.08 V en una noche quieta.
//
// Que fue un pico y no un voltaje real esta probado por eliminacion:
//
//   1. Los cinco corrieron loop() completo, asi que no fueron repasos por
//      temporizador: esos no llegan a loop().
//   2. Los cinco publicaron mqtt_reconnected, no boot. bootStatusPublished vive
//      en RTC_DATA_ATTR, que sobrevive al deep sleep pero se borra en cualquier
//      reset. Asi que no fueron resets. El unico boot de todo el log es el del
//      boton a las 10:38:18Z.
//   3. No queda otra fuente armada: fue el pin.
//   4. Y las cinco veces, medio segundo despues, el firmware leyo el pin y lo
//      declaro APAGADO: publico engine_off, nunca engine_on. Si el carro hubiera
//      energizado el cable de verdad, el pin seguiria arriba al terminar el
//      arranque. Ni una vez en cinco.
//
// Por que este circuito es vulnerable a eso:
//
//   - ext0 dispara con el nivel DIGITAL del pin (~1.5 V en el S3) mientras el
//     firmware exige PIN_ON_V = 2.5 V medidos por ADC. Entre 1.5 y 2.5 V el chip
//     despierta y el software lo lee apagado. Los dos umbrales no se hablan.
//   - El enganche vive en el dominio RTC y es asincrono: agarra un flanco de
//     microsegundos, no necesita que el nivel se sostenga.
//   - Y el filtro RC esta del lado equivocado de la resistencia serie. La
//     topologia es VBUS -> 47k -> nodo -> 68k -> GND, con el capacitor de 100 nF
//     en el NODO, y despues 510 ohm hasta el pin. O sea que el capacitor queda
//     510 ohm ANTES del pin: un pico acoplado en la pista corta entre la
//     resistencia y el pin practicamente no ve filtro. Del lado del pin solo hay
//     el Zener de 3.6 V y unos picofaradios.
//   - Ese nodo trabaja con unos 40 uA y ~28 kohm en paralelo: es una antena
//     pequena, pero es una antena.
//
// ARREGLO DE HARDWARE PENDIENTE, independiente de este parche: capacitor de
// 10-100 nF DIRECTO del pin a GND, del lado del pin de los 510 ohm. Con esa
// resistencia da 5-50 us de filtro justo donde hoy no hay nada, que es la escala
// del pico, y no estorba a los antirrebotes de 3 s y 20 s.
//
// Mientras tanto, en software: antes de gastar celda se exige que el pin se
// SOSTENGA por encima del umbral el mismo tiempo que ya pide el antirrebote de
// encendido. Si un arranque real hunde el VBUS por debajo del umbral y esto se
// equivoca, el repaso de parqueo ya arreglado lo recoge: la deteccion se atrasa
// como maximo PARKED_POLL_S, no se pierde.
//
// COSTO CONOCIDO: un despertar espurio ya no llega a tener MQTT, asi que no deja
// hora en el broker. Solo suma en rtcSpuriousExt0, que sale en el siguiente
// arranque real por sys/wake. Si manana amanece en spur=5 sin una sola
// publicacion, eso por si solo confirma que los picos son reales.
// ===========================================================================
static void wakeConfirmExt0(bool coldBoot) {
#if !TEST_DISABLE_SLEEP && !TEST_FORCE_PARKED
  if (coldBoot || wakeCause != ESP_SLEEP_WAKEUP_EXT0) {
    return;
  }

  uint32_t t0        = millis();
  bool     sostenido = false;

  while ((millis() - t0) < ON_DEBOUNCE_MS) {
    if (readPinVolts() < PIN_ON_V) {
      sostenido = false;
      break;
    }
    sostenido = true;
    watchdogFeed();
    delay(IGN_SAMPLE_MS);
  }

  if (!sostenido) {
    rtcSpuriousExt0++;
    SerialMon.printf("[PM] ext0 espurio #%u: pin=%.3fV no se sostuvo %lums -> de vuelta a dormir\n",
                     (unsigned)rtcSpuriousExt0, readPinVolts(),
                     (unsigned long)ON_DEBOUNCE_MS);
    // El mundo fisico dice que el carro sigue apagado: se deja la memoria RTC
    // coherente con eso antes de volver al parqueo.
    ignState = IGN_OFF;
    pmDeepSleep(true, PARKED_POLL_S);   // NO RETORNA
  }

  SerialMon.printf("[PM] ext0 confirmado: el pin se sostuvo %lums -> ignicion real\n",
                   (unsigned long)ON_DEBOUNCE_MS);
#else
  (void)coldBoot;
#endif
}

// Se llama en setup() despues de tryConnectMQTT(). Retenido a proposito: la
// gracia es poder mirar el ultimo despertar sin tener que estar suscrito en el
// momento exacto.
static void wakePublishForensics(float bootPinV) {
  if (!mqtt.connected()) {
    return;
  }

  char w[96];
  snprintf(w, sizeof(w), "cause=%d,pin=%.3f,boot=%lu,spur=%u,fixage=%lu",
           (int)wakeCause, bootPinV, (unsigned long)rtcBootCount,
           (unsigned)rtcSpuriousExt0, (unsigned long)fixAgeS());

  mqtt.publish(TOPIC_WAKE, w, true);
  SerialMon.printf("[PM] wake %s\n", w);
}
