# Protocolo CSV de telemetría

Este documento define los formatos CSV publicados por el tracker en MQTT.

El subscriber debe mantener compatibilidad hacia atrás: si recibe `v1`, debe parsearlo como formato histórico; si recibe `v2`, debe usar el campo `quality` para decidir cómo limpiar o completar los datos antes de reenviar a Traccar.

## v1 — formato histórico

Formato:

```text
v1,<device_id>,<fix>,<lat>,<lon>,<speed>,<alt>,<vsat>,<acc>,<ts_iso_utc>
```

Campos:

1. `version`: siempre `v1`.
2. `device_id`: identificador del dispositivo, por ejemplo `Lilygo`.
3. `fix`: tipo de fix GNSS reportado por el módem.
4. `lat`: latitud decimal.
5. `lon`: longitud decimal.
6. `speed`: velocidad reportada por GNSS en km/h.
7. `alt`: altitud reportada por GNSS en metros.
8. `vsat`: satélites visibles/usados según lectura del módem.
9. `acc`: precisión/HDOP según lectura usada por el firmware.
10. `ts_iso_utc`: timestamp UTC en ISO-8601.

Ejemplo:

```text
v1,Lilygo,3,4.650000,-74.080000,32.50,2578.4,10,1.00,2026-06-03T01:47:39Z
```

## v2 — posición válida con calidad de velocidad/altitud

Formato:

```text
v2,<device_id>,<fix>,<lat>,<lon>,<speed>,<alt>,<vsat>,<acc>,<quality>,<ts_iso_utc>
```

Diferencia principal frente a `v1`:

- El firmware solo publica `v2` si la posición es válida.
- `speed` puede venir vacío si la velocidad GNSS no es confiable.
- `alt` puede venir vacío si la altitud GNSS no es confiable.
- `quality` resume la validez de velocidad y altitud para ahorrar campos.

### Campo `quality`

`quality` es un bitmask decimal:

| Bit | Valor | Significado |
| --- | ---: | --- |
| 0 | 1 | Altitud válida |
| 1 | 2 | Velocidad válida |

Valores posibles:

| `quality` | Altitud | Velocidad |
| ---: | --- | --- |
| 0 | inválida | inválida |
| 1 | válida | inválida |
| 2 | inválida | válida |
| 3 | válida | válida |

Ejemplo con todo válido:

```text
v2,Lilygo,3,4.650000,-74.080000,32.50,2578.4,10,1.00,3,2026-08-14T22:45:00Z
```

Ejemplo con posición válida, velocidad inválida y altitud inválida:

```text
v2,Lilygo,3,4.650000,-74.080000,,,8,1.10,0,2026-08-14T22:45:00Z
```

Ejemplo con posición válida, velocidad inválida y altitud válida:

```text
v2,Lilygo,3,4.650000,-74.080000,,2578.4,8,1.10,1,2026-08-14T22:45:00Z
```

## Regla esperada en el subscriber

El subscriber debe decidir por versión:

- Si `version == "v1"`: usar el parser histórico. Recomendado: aplicar saneamiento antes de reenviar a Traccar para evitar velocidades negativas o altitudes centinela.
- Si `version == "v2"`: parsear `quality`.
  - Si `quality & 2`: usar `speed` como velocidad GNSS válida.
  - Si no `quality & 2`: no reenviar esa velocidad a Traccar; opcionalmente calcular velocidad derivada usando el punto anterior válido.
  - Si `quality & 1`: usar `alt` como altitud válida.
  - Si no `quality & 1`: no reenviar altitud a Traccar.

La velocidad derivada se debe implementar primero en el subscriber, no en el ESP32, para poder ajustar reglas sin reflashear firmware.
