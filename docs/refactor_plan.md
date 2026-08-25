# Plan de Refactorización: Migración a Programación Orientada a Objetos en C++ (GPS Tracker Logan)

Este documento ha sido generado para que tú y tu socio (a través de Notion AI) tengáis visibilidad total de los cambios propuestos en la arquitectura del firmware.

## Objetivo Principal
El objetivo de este refactor es **puramente arquitectónico**. El firmware actual funciona de manera excelente en producción. Vamos a transformar los archivos `.h` espagueti en **Clases de C++ verdaderas** (`.cpp` y `.h`), inyectando las dependencias necesarias.

> [!IMPORTANT]
> **Funcionalidad 100% Idéntica:** Este refactor NO alterará el comportamiento del dispositivo.

## User Review Required

Se requiere la aprobación del usuario para iniciar la refactorización descrita a continuación. Ningún archivo será modificado hasta que no hagas clic en "Proceed" o des tu aprobación explícita.

## Decisiones Críticas de Arquitectura (Parches de Diseño)

Gracias a una revisión exhaustiva del diseño original, hemos identificado y parcheado 4 huecos arquitectónicos críticos antes de escribir una sola línea de código:

### 1. Gestión de Memoria RTC (RTC_DATA_ATTR)
El ESP32 no permite persistir variables de instancia de clases en la memoria RTC. 
**Solución:** Agruparemos las variables RTC en estructuras `struct` simples (ej. `RtcPowerState`). Estas estructuras se instanciarán de forma estática y global en `main.cpp` con `RTC_DATA_ATTR`, y pasaremos una **referencia** de estas estructuras a los constructores de nuestras clases.

### 2. Flujo de Ejecución y Métodos `[[noreturn]]`
Las funciones que llaman a `esp_deep_sleep_start()` (como `pmDeepSleep()` y `pmParkedTick()`) **nunca retornan**. El modelo mental de "el loop llama a `powerManager.service()` y sigue" es falso si este decide dormir.
**Solución:** 
- Renombraremos estos métodos a nombres explícitos: `enterParkedAndSleep()` y `sleepNow()`.
- Usaremos el atributo estándar de C++ `[[noreturn]]` en sus declaraciones para que el compilador y el lector humano sepan que la ejecución termina ahí.

### 3. Resolviendo la Dependencia Circular
Actualmente, Energía llama a Telemetría (`serviceEvents()`, `publishPoint()`) y Telemetría necesita a GNSS. Si las referenciamos mutuamente en los constructores, creamos un huevo y la gallina, exponiéndonos a punteros nulos durante el `setup()`.
**Solución:** Desacoplaremos los constructores. `PowerManager` **no** guardará una referencia a `TelemetryManager` en su estado interno. En su lugar, cuando Energía necesite parquear o ejecutar un corte (y por ende publicar el último punto), recibirá la instancia de Telemetría explícitamente en el llamado del método:
`[[noreturn]] void enterParkedAndSleep(TelemetryManager& telemetry);`
Esto garantiza que la dependencia siempre sea válida en el momento de uso y elimina el ciclo en los constructores.

### 4. Aislamiento en `platformio.ini`
Los entornos de prueba de PlatformIO (`gnss_lab`, `adc_sense`, etc.) usan filtros manuales restando archivos específicos (`-<main.cpp>`). Si añadimos nuevos `.cpp` a la carpeta `src/`, se colarán en todos los binarios de prueba por culpa del comportamiento por defecto de PlatformIO (`+<*>`).
**Solución:** Actualizaremos `platformio.ini` para usar un enfoque de *whitelist* estricto en cada entorno:
`build_src_filter = +<*> -<*> +<archivo_deseado.cpp>`
Para el entorno `tracker`, explícitamente incluiremos `main.cpp` y nuestros tres nuevos managers.

### 5. Centralización de Constantes
Actualmente todos los `#define` de comportamiento (umbrales de batería, cadencias, calibraciones ADC) viven en `main.cpp`. Al separar el código en clases, estas perderán acceso a dichos valores.
**Solución:** Crearemos `config.h` para alojar todas las constantes de negocio y configuración, dejándolo como la única fuente de verdad para parámetros como `TEST_PULSE_S` y `PIN_ON_V`.

---

## Proposed Changes

A continuación se detalla la nueva estructura de archivos.

### Archivos Compartidos y Configuración

#### [NEW] [config.h](file:///c:/Users/julia/Documents/PlatformIO/Projects/GPS%20Tracker%20Logan/include/config.h)
- Único lugar para los `#define` (ej: `BAT_CUTOFF_V`, `PARKED_POLL_S`, `TEST_FORCE_PARKED`). Se añadirá una advertencia gigante sobre el peligro de dejar activos los flags de TEST en producción.

#### [NEW] [types.h](file:///c:/Users/julia/Documents/PlatformIO/Projects/GPS%20Tracker%20Logan/include/types.h)
- Definición de la estructura `GpsPoint`, los `enum` (`IgnState`, `PendingEvent`) y las estructuras de estado RTC (`RtcPowerState`, `RtcTelemetryState`, `RtcGnssState`).

#### [MODIFY] [platformio.ini](file:///c:/Users/julia/Documents/PlatformIO/Projects/GPS%20Tracker%20Logan/platformio.ini)
- Refactorización de todos los `build_src_filter` para usar listas blancas (`+<*> -<*> +<archivos>`) impidiendo que las nuevas clases rompan los entornos de prueba.

---

### Componente: Motor GNSS

#### [NEW] [GnssManager.h](file:///c:/Users/julia/Documents/PlatformIO/Projects/GPS%20Tracker%20Logan/include/GnssManager.h)
#### [NEW] [GnssManager.cpp](file:///c:/Users/julia/Documents/PlatformIO/Projects/GPS%20Tracker%20Logan/src/GnssManager.cpp)
- **Responsabilidad:** Encendido asíncrono y guardia de tramas rancias.
- **Inyección:** `TinyGsm& modem`, `RtcGnssState& rtcState`.
#### [DELETE] [gnss_prod.h](file:///c:/Users/julia/Documents/PlatformIO/Projects/GPS%20Tracker%20Logan/include/gnss_prod.h)

---

### Componente: Telemetría

#### [NEW] [TelemetryManager.h](file:///c:/Users/julia/Documents/PlatformIO/Projects/GPS%20Tracker%20Logan/include/TelemetryManager.h)
#### [NEW] [TelemetryManager.cpp](file:///c:/Users/julia/Documents/PlatformIO/Projects/GPS%20Tracker%20Logan/src/TelemetryManager.cpp)
- **Responsabilidad:** Orquestar captura GNSS y publicación MQTT.
- **Inyección:** `TinyGsm& modem`, `PubSubClient& mqtt`, `GnssManager& gnss`, `RtcTelemetryState& rtcState`.
#### [DELETE] [tracker_telemetry.h](file:///c:/Users/julia/Documents/PlatformIO/Projects/GPS%20Tracker%20Logan/include/tracker_telemetry.h)

---

### Componente: Gestión de Energía

#### [NEW] [PowerManager.h](file:///c:/Users/julia/Documents/PlatformIO/Projects/GPS%20Tracker%20Logan/include/PowerManager.h)
#### [NEW] [PowerManager.cpp](file:///c:/Users/julia/Documents/PlatformIO/Projects/GPS%20Tracker%20Logan/src/PowerManager.cpp)
- **Responsabilidad:** DTR del módem, ADC de ignición/batería, y *Deep Sleep* (`[[noreturn]]`).
- **Inyección:** `TinyGsm& modem`, `TinyGsmClient& netClient`, `PubSubClient& mqtt`, `RtcPowerState& rtcState`.
#### [DELETE] [tracker_pm.h](file:///c:/Users/julia/Documents/PlatformIO/Projects/GPS%20Tracker%20Logan/include/tracker_pm.h)

---

### Componente Principal

#### [MODIFY] [main.cpp](file:///c:/Users/julia/Documents/PlatformIO/Projects/GPS%20Tracker%20Logan/src/main.cpp)
- Orquestador maestro. Definirá las instancias estáticas de RTC, creará los gestores y manejará el flujo principal.

## Verification Plan
1. **Compilación y Linking:** Validar el entorno `tracker` y uno de prueba (`gnss_lab`) mediante PlatformIO.
2. **Revisión Humana:** Validar que ninguna llamada de retorno espere ejecución tras un método `[[noreturn]]`.
