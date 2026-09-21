# Tracker diagnostics v1

Recolector independiente para persistir en PostgreSQL las señales de diagnóstico que Terror Gris ya publica por MQTT.

## Alcance

- Se suscribe únicamente a `sys/#`, `event/#` y `ack/#`.
- No se suscribe a `telemetria`; las posiciones continúan en Traccar.
- Elimina `lat`, `lon`, `latitude` y `longitude` de payloads JSON antes de almacenarlos.
- Guarda historial en `tracker_diag.messages` y último estado por topic en `tracker_diag.states`.
- Registra el tamaño original del payload para medir el presupuesto de datos.
- Funciona en paralelo al subscriber de Traccar; una falla no interrumpe el tracking.

## Despliegue

1. Fusionar `compose.snippet.yml` dentro del Compose de Mosquitto.
2. Validar la configuración de Compose.
3. Construir y levantar únicamente `tracker-diagnostics`.
4. Confirmar suscripciones y escritura en PostgreSQL.
5. No reiniciar ni recrear Mosquitto, PostgreSQL o `subscriber-json-osmand`.

## Consultas iniciales

```sql
SELECT topic, count(*) AS messages, sum(payload_bytes) AS payload_bytes
FROM tracker_diag.messages
WHERE received_at >= now() - interval '7 days'
GROUP BY topic
ORDER BY payload_bytes DESC;
```

```sql
SELECT device_id, topic, last_seen_at, payload_bytes
FROM tracker_diag.states
ORDER BY last_seen_at DESC;
```

Las cifras de `payload_bytes` miden el payload MQTT y no incluyen el overhead de TCP/IP, MQTT o la red celular.
