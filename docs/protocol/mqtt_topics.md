# Topics MQTT y politica de QoS

Este documento describe los topics que usa el tracker y por que cada uno usa
un nivel de servicio distinto. El criterio es balancear confiabilidad y consumo
de datos moviles.

## Resumen

| Topic | Contenido | QoS | Retained | Volumen |
|---|---|---|---|---|
| `tracker/Lilygo/telemetria` | CSV de telemetria GPS (v1/v2) | 0 | No | Alto (cada 5 s) |
| `tracker/Lilygo/sys/lwt` | `online` / `offline` | 1 (will) | Si | Muy bajo |
| `tracker/Lilygo/sys/status` | Estado del firmware | 0 con retained | Si | Bajo |

## Por que la telemetria va en QoS 0

La telemetria sale cada 5 segundos. Con QoS 1 cada publicacion agrega un
`PUBACK` de vuelta y posibles retransmisiones cuando la red LTE esta inestable.
Perder un punto GPS aislado no afecta el seguimiento porque el siguiente llega
pocos segundos despues, y el subscriber ya aplica filtros de intervalo minimo y
movimiento minimo. Mantenerla en QoS 0 reduce trafico, latencia y consumo de
energia.

## Por que el LWT va en QoS 1 y retenido

El Last Will Testament se registra una sola vez, en el momento de conectar.
No genera trafico periodico. Al ser QoS 1 y retenido:

- el broker garantiza la entrega del `offline` a los suscriptores,
- cualquier cliente que se suscriba despues recibe el ultimo estado conocido,
- se puede auditar cuando el dispositivo desaparecio sin cerrar sesion.

El costo en datos es despreciable porque son mensajes puntuales.

## Ciclo de vida del estado

1. El firmware conecta declarando el will:

   ```text
   topic:    tracker/Lilygo/sys/lwt
   payload:  offline
   qos:      1
   retained: true
   ```

2. Si la conexion es exitosa, el firmware publica de inmediato:

   ```text
   tracker/Lilygo/sys/lwt = online   (retained)
   ```

3. Si el dispositivo pierde red, se queda sin bateria o se cuelga, el broker
   publica automaticamente el will:

   ```text
   tracker/Lilygo/sys/lwt = offline  (retained)
   ```

El broker detecta la caida cuando pasa aproximadamente 1.5 veces el keepalive
sin recibir trafico del cliente. Con keepalive de 60 s, el `offline` aparece en
un plazo aproximado de 90 s.

## Estados publicados en `sys/status`

| Valor | Significado |
|---|---|
| `boot` | El firmware arranco (o se reinicio) |
| `mqtt_connected` | Primera conexion MQTT de esta sesion de arranque |
| `mqtt_reconnected` | Reconexion despues de una caida |

Un `boot` inesperado en medio de una ruta indica reinicio por watchdog, por
escalamiento de recuperacion o por corte de energia.

## Parametros de conexion

| Parametro | Valor | Motivo |
|---|---|---|
| Keepalive MQTT | 60 s | Tolerar microcortes y cambios de celda LTE |
| Socket timeout | 15 s | Evitar bloqueos largos con sockets muertos |
| Backoff de reintento | 2 s -> 30 s | No saturar la red cuando la senal esta mala |

## Escalamiento de recuperacion

Cuando fallan intentos consecutivos de conexion MQTT, el firmware sube de capa
en vez de insistir sobre la misma:

| Fallos consecutivos | Accion |
|---|---|
| 3 | Reconectar red LTE / GPRS |
| 6 | Reiniciar el modem |
| 10 | Reiniciar el ESP32 |

Ademas hay un watchdog de hardware con timeout de 120 s. Si el firmware queda
bloqueado en cualquier punto, el ESP32 se reinicia por si solo.

## Comandos utiles de verificacion

Observar los topics de sistema:

```bash
mosquitto_sub -h localhost -t 'tracker/Lilygo/sys/#' -v
```

Observar la telemetria:

```bash
mosquitto_sub -h localhost -t 'tracker/Lilygo/telemetria' -v
```

Prueba de aceptacion del LWT:

1. Encender el tracker y confirmar `tracker/Lilygo/sys/lwt online`.
2. Cortar la alimentacion o la antena LTE.
3. Esperar aproximadamente 90 s.
4. Confirmar `tracker/Lilygo/sys/lwt offline`.
