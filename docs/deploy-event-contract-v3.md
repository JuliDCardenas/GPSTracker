# Despliegue seguro — contrato de ignición v3

## Orden obligatorio

1. Desplegar y validar el subscriber v3.
2. Confirmar que está suscrito a `tracker/Lilygo/event/ignition/v3`.
3. Solo después compilar y flashear el firmware.

Si se flashea primero, el tracker no recibirá ACK y conservará el evento pendiente.

## VPS: preparar el subscriber

La producción usa `/home/ubuntu/mosquitto/docker-compose.yml` y el contexto `./GPSTracker/server`.

### 1. Actualizar el repositorio

```bash
cd /home/ubuntu/mosquitto/GPSTracker
git status
git fetch origin
git switch fix/ignition-event-contract-v2.3
git pull --ff-only origin fix/ignition-event-contract-v2.3
```

No usar `reset --hard` si `git status` muestra cambios locales.

### 2. Persistir SQLite

En el servicio `subscriber-json-osmand`, agregar:

```yaml
    environment:
      EVENT_DB_PATH: /data/event_state.sqlite3
      MQTT_EVENT_TOPIC: tracker/Lilygo/event/ignition/v3
      MQTT_EVENT_ACK_TOPIC: tracker/Lilygo/ack/ignition/v3
    volumes:
      - subscriber_event_state:/data
```

En el bloque global `volumes:` agregar:

```yaml
  subscriber_event_state:
```

Conservar las variables MQTT y Traccar existentes; no copiar secretos al repositorio.

### 3. Reconstruir solo el subscriber

```bash
cd /home/ubuntu/mosquitto
docker compose config
docker compose up -d --build subscriber-json-osmand
docker compose ps subscriber-json-osmand
docker compose logs --tail=100 subscriber-json-osmand
```

No ejecutar `docker compose down`: no es necesario bajar Mosquitto, Traccar ni Postgres.

### 4. Comprobar suscripciones

El log debe incluir:

```text
Subscribed telemetry=tracker/Lilygo/telemetria events=tracker/Lilygo/event/ignition/v3
```

## PC: compilar y flashear

```bash
cd <carpeta-del-proyecto>
git status
git fetch origin
git switch fix/ignition-event-contract-v2.3
git pull --ff-only origin fix/ignition-event-contract-v2.3
pio run -e tracker -t clean
pio run -e tracker
pio run -e tracker -t upload --upload-port COM5
pio device monitor --baud 115200 --port COM5
```

El `clean` es obligatorio para que la fecha/hora de compilación corresponda al binario nuevo.

## Huella esperada

Serial y `tracker/Lilygo/sys/fw` deben mostrar:

```text
tracker 2.3.0-rc1,sha=<SHA de 8 caracteres>,build=<fecha y hora>
```

## Prueba mínima antes de instalar en el carro

1. Arrancar con el subscriber v3 activo.
2. Confirmar `subscribe ack qos=1 result=1`.
3. Producir una transición ON y observar un ID nuevo.
4. Confirmar `EVENT nuevo` en el subscriber y luego `ACK` en el firmware.
5. Reinyectar el mismo JSON y comprobar `EVENT duplicado`, sin segundo evento en Traccar.
6. Probar una transición sin fix y confirmar `position_source=none`.
7. No fusionar el PR hasta completar al menos un ciclo ON/OFF y un deep sleep de banco.
