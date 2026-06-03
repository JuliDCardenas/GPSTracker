# Telemetry CSV protocol

## v1 (tag: csv-v1.0)
Formato:
v1,<device_id>,<fix>,<lat>,<lon>,<speed>,<alt>,<vsat>,<acc>,<ts_iso_utc>

Campos:
1. version: "v1"
2. device_id: string (ej. Lilygo)
...
Ejemplo:
v1,Lilygo,3,4.65,-74.08,32.5,2578.4,10,1.00,2026-06-03T01:47:39Z
