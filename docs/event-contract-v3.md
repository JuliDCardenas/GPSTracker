# Contrato de eventos de ignición v3

## Objetivo
Separar una transición física de ignición de la disponibilidad de GNSS y volver su entrega persistente e idempotente.

## Topics
- Estado retenido: `tracker/Lilygo/state/ignition` (`on` o `off`).
- Evento no retenido: `tracker/Lilygo/event/ignition/v3` (JSON).
- ACK: `tracker/Lilygo/ack/ignition/v3` (`event_id` como texto).
- Compatibilidad temporal: `tracker/Lilygo/event/ignition` conserva el estado retenido legado y no debe interpretarse como evento lógico.

## Semántica
El evento contiene `event_id`, tipo, ignición, instante monotónico, `position_source=fresh|cache|none`, `fix_age_s` y una posición opcional. Si no existe posición, `fix_age_s` es `null` y se omite el objeto `position`.

El ACK confirma que el subscriber guardó el evento durablemente en SQLite. No significa que Traccar ya lo aceptó. Después del ACK, el tracker puede limpiar el registro RTC; el subscriber conserva la responsabilidad de reintentar Traccar.

El subscriber usa `event_id` como clave primaria. Una republicación con el mismo ID recibe nuevamente ACK, pero no genera un segundo evento en Traccar.

## Persistencia
El firmware conservará una cola RTC de hasta ocho transiciones. El subscriber conserva la inbox, el estado de entrega y la última posición válida. La ruta predeterminada es `/data/event_state.sqlite3`; producción debe montar `/data` en un volumen persistente.

## Criterios de aceptación
1. ON/OFF se captura antes de consultar GNSS o red.
2. Un evento sin posición llega al subscriber y recibe ACK.
3. Todos los reintentos reutilizan el mismo `event_id`.
4. Un duplicado no vuelve a enviarse a Traccar.
5. Una posición cacheada lleva su fuente y edad real.
6. Las transiciones pendientes sobreviven deep sleep.
