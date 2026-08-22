# GPS Tracker Logan — historia y reglas del firmware

Este documento salió de la cabecera de comentarios de `src/main.cpp`. Se movió
aquí el 2026-08-22 porque el archivo llegó a 67 KB y ya no se podía reescribir
de una sola pasada con herramientas remotas. **No es documentación decorativa:**
cada párrafo es una noche de pruebas y varios son la explicación de por qué una
línea de código está escrita como está. Antes de "limpiar" algo del firmware,
búsquelo aquí.

## Qué hace el firmware principal (env: `tracker`)

Telemetría GNSS por LTE hacia MQTT + sense de ignición por ADC en GPIO9.

El estado de la ignición gobierna la cadencia de publicación:

| Estado | Cadencia |
|---|---|
| `IGN_ON` en marcha | cada `GPS_PERIOD_MS` (5 s) |
| `IGN_ON` en ralentí | cada `IDLE_PERIOD_MS` (30 s) |
| `IGN_OFF` parqueado | `engine_off` y luego deep sleep (Nivel 2): repaso de ignición cada 120 s y un pulso de batería/posición al día |

Y además genera dos eventos discretos en las transiciones: `engine_on` y
`engine_off`. El de apagado se publica con la última posición válida guardada en
la caché, de modo que el "aquí quedó parqueado" sale aunque el GNSS no tenga fix
fresco en ese instante.

## Nivel 1 de ahorro de energía

Aquí no duerme nada (ni el ESP32, ni el módem, ni el GNSS). Lo único que baja es
la cadencia de publicación. Se eligió así porque con este stack (TinyGSM +
PubSubClient + LTE) volver a despertar cuesta 30-60 s de re-attach a plena
potencia: pulso de PWRKEY, `waitForNetwork`, `gprsConnect`, `mqtt.connect` y el
TTFF del GNSS. Con intervalos cortos, dormir sale más caro que quedarse
despierto.

**Medido** (noche del 2026-08-17 al 18, 8 h 22 min parqueado con keepalive de
20 min): 25 de 25 keepalives entregados, cero reconexiones MQTT, y la 18650 bajó
de ~4.11 V a 3.79 V, unos 38 mV/h. Autonomía estimada del Nivel 1: 15-18 h. Con
solo 25 transmisiones en 8 horas, el intervalo del keepalive es irrelevante para
el consumo: lo que cuesta es tener el módem enganchado y el GNSS encendido. El
siguiente salto real es el Nivel 2, no publicar menos seguido.

## Nivel 2 de ahorro de energía

Parqueado, el firmware publica el `engine_off`, apaga el GNSS (`AT+CGNSSPWR=0`),
duerme el módem por DTR (`AT+CSCLK=1`; conserva el registro LTE y las
efemérides) y pone el ESP32 en deep sleep. Despierta por `ext0` si la ignición
sube y, como red de seguridad, cada 120 s por temporizador. Una vez al día
(`PARKED_PULSE_S`) despierta el módem, publica batería + posición y vuelve a
dormir.

El corte por bajo voltaje (`BAT_CUTOFF_V`, con histéresis `BAT_RECOVER_V`) apaga
TODO con `AT+CPOF` y solo rearma con la celda recuperada o con VBUS presente. El
guardián de arranque bloquea cualquier boot sin VBUS por debajo del corte, sin
encender el módem: eso mata el bucle de brownout del 2026-08-18 (10 reconexiones
en 108 s, muerte a 2.37 V).

**Medido** (noche del 2026-08-19 al 20, 5 h 35 min parqueado con `TEST_PULSE_S`
de una hora): 5 de 5 pulsos entregados, caché de posición y contadores RTC
intactos toda la noche, y la 18650 bajó de 4.15 V a 4.10 V en 4 h 28 min, unos
11.2 mV/h contra los 38 mV/h del Nivel 1.

**Ojo al leer esa cifra:** 4.19-4.10 V es la zona más plana de la curva de la
18650, así que en mV/h el Nivel 2 se ve mejor de lo que es. Traducido a capacidad
son ~1.25 %/h contra ~2.87 %/h del Nivel 1: una mejora real de **2.3x, no de
3.4x**. La medición honesta pide dejarla correr hasta 3.80 V.

Y esa noche todavía cargaba el desperdicio de la espera del CDC USB (3.3 s de CPU
a plena potencia en cada repaso de 30 s, un 10 % de ciclo útil), que ya está
corregido. Ver el comentario de `SERIAL_CDC_WAIT_MS`.

## Entregado a MQTT no es entregado a Traccar

Lección del 2026-08-20 por la mañana. Esos 5 pulsos sí estaban en el broker, pero
el subscriber los tiró todos con `Skip: no_move_0.0m` y la noche entera quedó
invisible en el mapa. El CSV decía `ignition=1` porque `ignState` no sobrevivía
al deep sleep, y sin `ignition=0` no hay bypass del filtro de movimiento.

**Una prueba de campo no termina en `mosquitto_sub`: termina en el log del
contenedor.**

## Ojo con el retraso de detección

Los 120 s del repaso NO son la latencia de detección de la ignición. El despertar
real lo hace `ext0`, que es una interrupción de hardware en GPIO9 y es
instantánea. El temporizador existe como red de seguridad (si `ext0` no se pudo
armar) y para vigilar el voltaje de la celda mientras duerme. Al encender el
carro, lo que se demora son el antirrebote (3 s) y el re-attach de LTE+MQTT
(~5-15 s), no el sondeo.

## Las siete reglas

### Primera — regla de oro del Nivel 2 (2026-08-20)

De `pmDeepSleep()` no se sale nunca sin una fuente de despertar armada. Un deep
sleep sin `ext0` y sin temporizador no es un ahorro de energía, es un ladrillo
que solo revive desconectando la batería.

### Segunda — misma noche, mismo precio

Si el firmware puede quedarse varios minutos sin publicar y sin imprimir,
entonces un cuelgue y una espera normal se ven EXACTAMENTE igual, y se depura a
ciegas adivinando. Todo estado que dure más de unos segundos tiene que
anunciarse. Ver el log periódico del `loop()` y `TEST_DISABLE_SLEEP`.

### Tercera — la que costó toda la noche

Ninguna decisión irreversible se toma con una sola muestra del ADC. Las primeras
conversiones tras el arranque son basura y casi dejaron esta rama por
inservible. Ver `adcSetup()` y `pmBootGuard()`.

### Cuarta — la mañana siguiente

En parqueo, cada milisegundo despierto se paga con celda. Todo lo que se agregue
al camino de arranque corre cientos de veces al día, así que ninguna espera va
antes de saber POR QUÉ despertamos. La observabilidad se agrega para el arranque
en frío, no para el repaso.

### Quinta — media hora después de la cuarta

Todo estado del que dependa una publicación tiene que vivir en `RTC_DATA_ATTR` o
recalcularse en el despertar. Un deep sleep no es una pausa: es un reset con
memoria selectiva, y las variables normales vuelven a su valor inicial. Ver
`ignState`.

### Sexta — banco del 2026-08-22

**Que una librería devuelva `false` no significa que la operación falló.**

`TinyGSM::enableGPS()` manda `AT+CGNSSPWR=1` y espera hasta 30 segundos el URC
`+CGNSSPWR: READY!`. Ese URC **no existe** en el firmware
`SIM767XM5_B05V01_241206`; comprobado en tres corridas del laboratorio GNSS
(v1.1, v1.2 y v1.4). El módem contesta `OK` en unos 70 ms y el motor queda
operativo a los ~2 s, pero `enableGPS()` devuelve `false` porque nunca ve su URC.

Costo real de creerle a ese `false`: `pmGnssOn()` reintentaba 20 veces, y
20 × (30 s + 0,5 s) son **más de diez minutos** de `setup()` bloqueado en cada
arranque. Diez minutos sin muestrear la ignición, sin reintentar MQTT y sin
publicar una sola posición. El watchdog de 120 s no salta porque el bucle
alimentaba `watchdogFeed()` en cada vuelta: el firmware se colgaba "bien
portado". Ver `include/gnss_prod.h`.

### Séptima — mismo banco

**Una lectura con buenos números no es una lectura nueva. Solo la marca de tiempo
delata la trama vieja.**

Laboratorio v1.4, corrida T10: después de `AT+CGNSSWAKEUP` el primer
`+CGNSSINFO` devolvió un fix con 27 satélites y HDOP 1,4 y marca de tiempo
`220826/165136.000` — exactamente la del fix tomado 150 segundos ANTES. El fix
real llegó 45 segundos después, con marca `220826/165452.000`.

Lo peligroso es que la trama vieja se veía **mejor** que las reales: 27 sats
contra 26, HDOP 1,4 contra 1,6. Ningún filtro de calidad la atrapa. Sin guardia,
T10 habría reportado un TTFF de 0 segundos.

## Pendientes conocidos

- **Unidad de velocidad de `AT+CGNSSINFO`** (F20): la evidencia de campo apunta a
  NUDOS y no km/h. El protocolo OsmAnd de Traccar espera nudos, así que si el
  módem da nudos el pipeline acierta por accidente. No cambiar el CSV hasta
  cerrarlo contra el velocímetro. Hay instrumentación en `gnssLogSpeedUnits()`.
- **A-GNSS**: sin cerrar. `AT+CAGPS` responde `OK` pero no es el comando
  documentado; el correcto es `AT+CGNSSAGPS` (§21.2.23 del manual V1.02), que
  puede contestar `OK` habiendo fallado. La única medición dio 192 s con `sats=9`
  usando el comando equivocado.
- **Dormir el chip GNSS vs apagarlo**: `CGNSSSLEEP`/`CGNSSWAKEUP` funcionan, pero
  la única medición dio TTFF peor (45 s contra 22 s). n=1, sin cerrar.
- **Socket zombi `state=-4`**: el primer `mqtt.connect()` tras encender el módem
  falla de forma reproducible y el segundo conecta. El backoff lo absorbe, pero
  en el Nivel 2 ese reintento se paga con batería.
- **`engine_on` con posición vieja**: falta un límite de antigüedad para la caché
  en ese evento. El `engine_off` sí la quiere siempre.
- **Hardware**: bajar el divisor de ignición a 10k/12k y ajustar el pot del
  MP1584 de 5.25 V a 5.05-5.10 V. Habrá que recalibrar `DIVIDER_FACTOR` y bajar
  el umbral ON a 2.2 V.
