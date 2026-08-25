# Plan de Refactorización: Migración a Programación Orientada a Objetos en C++ (GPS Tracker Logan)

Este documento ha sido generado para que tú y tu socio (a través de Notion AI) tengáis visibilidad total de los cambios propuestos en la arquitectura del firmware.

## Objetivo Principal
El objetivo de este refactor es **puramente arquitectónico**. El firmware actual funciona de manera excelente en producción y la lógica de negocio (gestión de energía, reportes GNSS, lógica de ignición y LWT MQTT) ha sido probada exhaustivamente. 

Actualmente, el código sufre de un antipatrón en el que la lógica de negocio vive en archivos `.h` (como `tracker_pm.h` o `tracker_telemetry.h`) que se incrustan en medio de `main.cpp` para acceder a variables globales. Vamos a transformar esos archivos en **Clases de C++ verdaderas** (`.cpp` y `.h`), inyectando las dependencias necesarias.

> [!IMPORTANT]
> **Funcionalidad 100% Idéntica:** Este refactor NO alterará el comportamiento del dispositivo. Los tópicos MQTT, el manejo de la batería, las reglas de ignición y los tiempos de latencia del Nivel 2 de energía seguirán funcionando **exactamente igual**. Solo cambiará la organización interna del código.

## User Review Required

Se requiere la aprobación del usuario para iniciar la refactorización descrita a continuación. Ningún archivo será modificado hasta que no hagas clic en "Proceed" o des tu aprobación explícita.

## ⚠️ CRÍTICO: Gestión de Memoria RTC (RTC_DATA_ATTR)
Como bien identificó el equipo, el ESP32 no permite persistir variables de instancia de clases en la memoria RTC (RTC Slow Memory). Si las variables de estado pasan a ser campos ordinarios de una clase C++, se alojarán en `.bss` o en el *heap* y se destruirán durante el *deep sleep*, rompiendo toda la lógica del Nivel 2.

Para solucionar esto manteniendo una arquitectura orientada a objetos, aplicaremos el **Patrón de Estado Inyectado**:
1. Agruparemos las variables RTC en estructuras `struct` simples (Plain Old Data - POD).
2. Estas estructuras se instanciarán de forma estática en `main.cpp` o en un módulo dedicado, con el atributo `RTC_DATA_ATTR`.
3. Pasaremos una **referencia** de estas estructuras a los constructores de nuestras clases.

Ejemplo de cómo se verá:
```cpp
// En types.h
struct RtcPowerState {
    bool modemAlive;
    bool inCutoff;
    uint32_t bootCount;
    // ...
};

// En main.cpp
static RTC_DATA_ATTR RtcPowerState rtcPower = {0};

// En PowerManager.cpp, la clase accede a _rtcState.modemAlive
PowerManager::PowerManager(TinyGsm& modem, RtcPowerState& state) : _rtcState(state) {}
```
De esta forma, las clases se mantienen puras (sin dependencias globales) pero mutan directamente la memoria RTC que sobrevive al sueño.

## Proposed Changes

A continuación se detalla cómo se reorganizará el código fuente.

### Estructuras de Datos Globales y RTC
Se creará un archivo compartido para definir los tipos básicos y las estructuras de estado RTC.

#### [NEW] [types.h](file:///c:/Users/julia/Documents/PlatformIO/Projects/GPS%20Tracker%20Logan/include/types.h)
- Contendrá la definición de la estructura `GpsPoint`.
- Contendrá los `enum` relacionados (como `IgnState` y `PendingEvent`).
- Contendrá las estructuras `RtcPowerState`, `RtcTelemetryState` y `RtcGnssState`.

---

### Componente: Motor GNSS
Se migrará `gnss_prod.h` a una clase.

#### [NEW] [GnssManager.h](file:///c:/Users/julia/Documents/PlatformIO/Projects/GPS%20Tracker%20Logan/include/GnssManager.h)
#### [NEW] [GnssManager.cpp](file:///c:/Users/julia/Documents/PlatformIO/Projects/GPS%20Tracker%20Logan/src/GnssManager.cpp)
- **Responsabilidad:** Encender el módulo GPS sin bloquear, obtener fixes y gestionar la detección de "tramas rancias".
- **Dependencias Inyectadas:** `TinyGsm& modem` y `RtcGnssState& rtcState` (para `rtcGnssStale`).
#### [DELETE] [gnss_prod.h](file:///c:/Users/julia/Documents/PlatformIO/Projects/GPS%20Tracker%20Logan/include/gnss_prod.h)

---

### Componente: Telemetría
Se migrará `tracker_telemetry.h` a su respectiva clase.

#### [NEW] [TelemetryManager.h](file:///c:/Users/julia/Documents/PlatformIO/Projects/GPS%20Tracker%20Logan/include/TelemetryManager.h)
#### [NEW] [TelemetryManager.cpp](file:///c:/Users/julia/Documents/PlatformIO/Projects/GPS%20Tracker%20Logan/src/TelemetryManager.cpp)
- **Responsabilidad:** Orquestar la captura de datos GNSS, formato CSV e interactuar con el cliente MQTT para su publicación.
- **Dependencias Inyectadas:** `TinyGsm& modem`, `PubSubClient& mqtt`, referencia a `GnssManager` y `RtcTelemetryState& rtcState` (para `lastValidPoint`, `pendingEvent`, `ignState`, `bootStatusPublished`).
#### [DELETE] [tracker_telemetry.h](file:///c:/Users/julia/Documents/PlatformIO/Projects/GPS%20Tracker%20Logan/include/tracker_telemetry.h)

---

### Componente: Gestión de Energía (Power Management)
Se migrará `tracker_pm.h` aislando la gestión de *deep sleep* y batería.

#### [NEW] [PowerManager.h](file:///c:/Users/julia/Documents/PlatformIO/Projects/GPS%20Tracker%20Logan/include/PowerManager.h)
#### [NEW] [PowerManager.cpp](file:///c:/Users/julia/Documents/PlatformIO/Projects/GPS%20Tracker%20Logan/src/PowerManager.cpp)
- **Responsabilidad:** Lectura del voltaje (ADC), control del pin DTR, corte por bajo voltaje y *Deep Sleep* en Nivel 2.
- **Dependencias Inyectadas:** `TinyGsm& modem`, `TinyGsmClient& netClient`, `PubSubClient& mqtt`, y `RtcPowerState& rtcState`.
#### [DELETE] [tracker_pm.h](file:///c:/Users/julia/Documents/PlatformIO/Projects/GPS%20Tracker%20Logan/include/tracker_pm.h)

---

### Componente Principal (Main)
El orquestador final.

#### [MODIFY] [main.cpp](file:///c:/Users/julia/Documents/PlatformIO/Projects/GPS%20Tracker%20Logan/src/main.cpp)
- **Cambios:** Instanciación de las estructuras `RTC_DATA_ATTR` estáticas. Se inicializarán los gestores (`PowerManager`, `TelemetryManager`, `GnssManager`) pasándoles las instancias de RTC y dependencias de red.

## Verification Plan

### Manual Verification
1. **Compilación Exitosa:** Usaremos PlatformIO para garantizar que la reestructuración compila perfectamente y los enlaces (*linking*) son correctos.
2. **Supervivencia del Estado RTC:** El usuario deberá verificar mediante *logging* que, tras un ciclo de parqueo, el tracker despierta conservando su `rtcBootCount` y su estado de módem.
3. **Ejecución y Mantenimiento de Comportamiento:** Una vez flasheado el dispositivo por el usuario, se deberá verificar que se reporten posiciones válidas vía MQTT como antes y que el Nivel 2 de energía opere sin alteraciones.
