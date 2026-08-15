# GPS Tracker Logan

Rastreador GPS vehicular basado en LilyGO T-SIM7670G (ESP32-S3 + modem LTE).
El dispositivo publica telemetria GNSS por MQTT, un subscriber en Python la
normaliza y la reenvia a Traccar usando el protocolo OsmAnd.

## Arquitectura

```text
LilyGO T-SIM7670G  ->  LTE  ->  Mosquitto  ->  subscriber Python  ->  Traccar
   (firmware)                    (broker)        (normaliza/filtra)     (mapa)
```

| Capa | Componente | Ubicacion |
|---|---|---|
| Dispositivo | Firmware ESP32-S3 + SIM7670G | `src/main.cpp` |
| Transporte | MQTT sobre LTE | `tracker/Lilygo/#` |
| Servidor | Subscriber MQTT -> Traccar | `server/subscriberJsonOsmAnd.py` |
| Visualizacion | Traccar (protocolo OsmAnd) | externo |

## Estructura del repositorio

```text
src/                        Firmware y utilidades de diagnostico
  main.cpp                  Firmware principal del tracker
  board_check.cpp           Prueba basica de placa
  at_passthrough.cpp        Consola AT directa al modem
  mqtt_smoke.cpp            Prueba minima de conectividad MQTT
server/                     Subscriber que alimenta a Traccar
  subscriberJsonOsmAnd.py   Parser CSV v1/v2 y puente a Traccar
  Dockerfile
  requirements.txt
docs/protocol/              Contratos de datos
  telemetry_csv.md          Formato CSV v1 y v2
  mqtt_topics.md            Topics de sistema y politica de QoS
platformio.ini              Entornos de compilacion
```

## Firmware

### Compilar y cargar

El proyecto usa PlatformIO con varios entornos independientes:

```bash
# Firmware principal
pio run -e tracker -t upload
pio device monitor -b 115200

# Diagnostico
pio run -e board_check -t upload      # verificar la placa
pio run -e at_passthrough -t upload   # hablar AT con el modem
pio run -e mqtt_smoke -t upload       # probar solo conectividad MQTT
```

Ajusta `upload_port` en `platformio.ini` segun tu sistema.

### Que hace el firmware

1. Enciende el modem y espera respuesta AT.
2. Levanta la red LTE y el contexto GPRS con el APN configurado.
3. Conecta a MQTT declarando un Last Will Testament.
4. Habilita el GNSS.
5. Cada 5 segundos lee posicion y publica telemetria en CSV v2.

### Validacion GNSS en el dispositivo

El firmware solo publica cuando la posicion es confiable:

| Criterio | Umbral |
|---|---|
| Fix | 2D o 3D |
| Satelites | minimo 5 |
| HDOP | maximo 2.5 |
| Coordenadas | dentro de rango y distintas de 0,0 |

La velocidad y la altitud se validan por separado. Si alguna no es confiable,
se publica vacia y se refleja en el campo `quality`. La posicion nunca se
descarta por culpa de una velocidad o altitud dudosa.

### Robustez de conexion

| Mecanismo | Valor |
|---|---|
| Keepalive MQTT | 60 s |
| Socket timeout | 15 s |
| Backoff de reintento | 2 s a 30 s |
| Watchdog de hardware | 120 s |

Escalamiento por fallos consecutivos de MQTT:

| Fallos | Accion |
|---|---|
| 3 | Reconectar LTE / GPRS |
| 6 | Reiniciar el modem |
| 10 | Reiniciar el ESP32 |

La reconexion no bloquea el loop principal, asi que el GPS y el watchdog siguen
activos mientras el enlace se recupera.

## Protocolo de telemetria

Formato actual (CSV v2):

```text
v2,<device_id>,<fix>,<lat>,<lon>,<speed>,<alt>,<vsat>,<acc>,<quality>,<ts_iso_utc>
```

El campo `quality` es un bitmask:

| Valor | Altitud | Velocidad |
|---|---|---|
| 0 | invalida | invalida |
| 1 | valida | invalida |
| 2 | invalida | valida |
| 3 | valida | valida |

El formato v1 sigue soportado por el subscriber. Detalle completo en
`docs/protocol/telemetry_csv.md`.

### Topics

| Topic | Contenido | QoS |
|---|---|---|
| `tracker/Lilygo/telemetria` | Telemetria GNSS | 0 |
| `tracker/Lilygo/sys/lwt` | `online` / `offline` | 1, retenido |
| `tracker/Lilygo/sys/status` | `boot`, `mqtt_connected`, `mqtt_reconnected` | retenido |

Detalle y justificacion en `docs/protocol/mqtt_topics.md`.

## Servidor

El subscriber se ejecuta en el VPS junto a Mosquitto y Traccar.

```bash
cd ~/mosquitto/GPSTracker
docker compose up -d subscriber-json-osmand
docker compose logs -f subscriber-json-osmand
```

### Responsabilidades

- Acepta CSV v1, CSV v2 y el formato v2 legado con evento.
- Sanea velocidad y altitud fuera de rango.
- Deriva velocidad por distancia y tiempo cuando el GNSS no la reporta.
- Aplica intervalo minimo y movimiento minimo para no saturar Traccar.
- Reenvia por HTTP al puerto OsmAnd de Traccar.

### Variables de entorno

| Variable | Default | Descripcion |
|---|---|---|
| `MQTT_HOST` | `mqtt.julidcardenas.site` | Host del broker |
| `MQTT_PORT` | `1883` | Puerto del broker |
| `MQTT_USER` | | Usuario MQTT |
| `MQTT_PASS` | | Password MQTT |
| `MQTT_TOPIC` | `tracker/Lilygo/telemetria` | Topic de telemetria |
| `TRACCAR_HOST` | `traccar.julidcardenas.site` | Host de Traccar |
| `TRACCAR_PORT` | `5055` | Puerto OsmAnd |
| `TRACCAR_DEVICE_ID` | `Lilygo` | Identificador en Traccar |
| `MIN_INTERVAL_SEC` | `10` | Intervalo minimo entre envios |
| `MIN_MOVE_METERS` | `20` | Movimiento minimo para reportar |
| `HTTP_TIMEOUT_SEC` | `5` | Timeout HTTP hacia Traccar |
| `MAX_VALID_SPEED_KMH` | `180` | Velocidad maxima aceptada |
| `MIN_VALID_ALTITUDE_M` | `-9990` | Altitud minima aceptada |
| `MIN_DERIVED_DT_SEC` | `2` | Delta minimo para derivar velocidad |
| `MAX_DERIVED_DT_SEC` | `120` | Delta maximo para derivar velocidad |
| `MAX_DERIVED_SPEED_KMH` | `180` | Tope de velocidad derivada |

### Lectura de logs

```text
TRACCAR OK ver=v2 lat=4.69023 lon=-74.07161 speed=18.65 src=gps alt=2572.0 q=3
Skip: rate_limit
Skip: no_move_1.9m
```

| Mensaje | Significado |
|---|---|
| `TRACCAR OK` | Punto aceptado y enviado a Traccar |
| `Skip: rate_limit` | Llego antes del intervalo minimo |
| `Skip: no_move_Xm` | El vehiculo no se movio lo suficiente |
| `src=gps` | Velocidad reportada por el GNSS |
| `src=derived` | Velocidad calculada en el servidor |

## Diagnostico

Cuando se corta el flujo de datos, revisar en este orden:

1. **Subscriber**: si no hay ninguna linea, no llegaron mensajes MQTT.

   ```bash
   docker compose logs --since "2026-08-15T05:40:00" subscriber-json-osmand
   ```

2. **Broker**: buscar desconexiones del cliente del tracker.

   ```bash
   docker compose logs mosquitto | grep logan-
   ```

   `exceeded timeout` indica que el dispositivo dejo de responder al keepalive.
   `session taken over` indica una reconexion con el mismo `clientId`.

3. **Estado del dispositivo**: consultar los topics retenidos.

   ```bash
   mosquitto_sub -h localhost -t 'tracker/Lilygo/sys/#' -v
   ```

Los timestamps de Mosquitto son Unix. Para convertirlos:

```bash
TZ=America/Bogota date -d @1786790804
```

## Ramas y tags

| Rama | Uso |
|---|---|
| `main` | Base historica |
| `stable/mqtt-gps-v1-working` | Rama activa de trabajo |

| Tag | Contenido |
|---|---|
| `csv-v1.0` | Protocolo CSV v1 |
| `csv-v2.0` | CSV v2 con bitmask de calidad |
| `csv-v2.1` | Subscriber compatible v1/v2 y velocidad derivada |

## Estado y siguientes pasos

- [x] Telemetria GNSS end to end hasta Traccar
- [x] CSV v2 con bitmask de calidad
- [x] Subscriber compatible con v1 y v2
- [x] Velocidad derivada en el servidor
- [x] LWT, keepalive extendido y reconexion escalonada
- [ ] Prueba en ruta del LWT y la reconexion
- [ ] Cola offline en flash para rutas sin cobertura
- [ ] Deteccion de ignicion y eventos de vehiculo
