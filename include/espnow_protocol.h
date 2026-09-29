#pragma once
// ==========================================================================
// espnow_protocol.h — Protocolo ESP-NOW v2 compartido entre LilyGO y SmallTV
//
// ESTE ARCHIVO SE COPIA IDENTICO A AMBOS PROYECTOS.
// Cualquier cambio debe reflejarse en las dos copias.
//
// Proyecto: GPS Tracker Logan + SmallTV Ultra Display
// Rama: feat/espnow-smalltv-display
// ==========================================================================

#include <stdint.h>
#include <string.h>

// ---- Versión de protocolo ----
// Incrementar si cambia el layout de CUALQUIER estructura.
#define ESPNOW_PROTOCOL_VERSION  2

// ---- Canal fijo compartido ----
// Ambos dispositivos deben usar este canal. NO depende de ningún router.
// Canal 1 elegido por ser el predeterminado y el de menor interferencia en
// el contexto vehicular (sin redes domésticas alrededor).
#define ESPNOW_CHANNEL           1

// ---- Tipos de mensaje ----
#define ESPNOW_MSG_FAST_TELEM    0x01
#define ESPNOW_MSG_HEALTH        0x02
#define ESPNOW_MSG_EVENT         0x03

// ---- Banderas de validez (bitmask en EspNowHeader.flags) ----
#define ESPNOW_VALID_SPEED       (1 << 0)
#define ESPNOW_VALID_RPM         (1 << 1)
#define ESPNOW_VALID_HEADING     (1 << 2)
#define ESPNOW_VALID_ALTITUDE    (1 << 3)
#define ESPNOW_VALID_TEMP_ECT    (1 << 4)
#define ESPNOW_VALID_VOLTAGE     (1 << 5)
#define ESPNOW_VALID_GNSS_FIX    (1 << 6)
#define ESPNOW_VALID_BATTERY     (1 << 7)

// ---- Fuentes de datos ----
#define ESPNOW_SOURCE_NONE       0
#define ESPNOW_SOURCE_GPS        1
#define ESPNOW_SOURCE_OBD        2
#define ESPNOW_SOURCE_ADC        3
#define ESPNOW_SOURCE_INTERNAL   4

// ---- Estados del tracker ----
#define ESPNOW_STATE_BOOT        0
#define ESPNOW_STATE_MOVING      1
#define ESPNOW_STATE_IDLE        2
#define ESPNOW_STATE_PARKED      3
#define ESPNOW_STATE_RETRY       4
#define ESPNOW_STATE_CUTOFF      5

// ---- Tipos de evento ----
#define ESPNOW_EVT_ENGINE_ON     1
#define ESPNOW_EVT_ENGINE_OFF    2
#define ESPNOW_EVT_LOW_BATTERY   3
#define ESPNOW_EVT_WAKE          4
#define ESPNOW_EVT_RESET         5
#define ESPNOW_EVT_MQTT_FAIL     6
#define ESPNOW_EVT_ACK_OK        7
#define ESPNOW_EVT_PARKED_SLEEP  8

// ---- Valor centinela de antigüedad ----
#define ESPNOW_AGE_UNKNOWN       255

// ---- Valor centinela de RPM no disponible ----
#define ESPNOW_RPM_UNAVAILABLE   0xFFFF

// ==========================================================================
// Header común (8 bytes)
// ==========================================================================
struct __attribute__((packed)) EspNowHeader {
  uint8_t  version;    // ESPNOW_PROTOCOL_VERSION
  uint8_t  msgType;    // ESPNOW_MSG_*
  uint16_t sequence;   // Secuencia global incremental
  uint8_t  length;     // Longitud total del paquete (header + payload)
  uint8_t  flags;      // Validez de campos (bitmask ESPNOW_VALID_*)
  uint8_t  ageDs;      // Antigüedad del dato más viejo, décimas de segundo
                       // 0 = fresco, 1-254 = 0.1-25.4 s, 255 = desconocido
  uint8_t  crc8;       // CRC-8 de todo el paquete (este campo = 0 al calcular)
};

// ==========================================================================
// Fast Telemetry — 2 Hz en MOVING, 1 Hz en IDLE (18 bytes)
// ==========================================================================
struct __attribute__((packed)) EspNowFastTelem {
  EspNowHeader hdr;     // 8 bytes
  int16_t  speedX10;    // Velocidad × 10 en km/h (0-2200 → 0.0-220.0 km/h)
  uint16_t rpm;         // RPM motor (0-8000). ESPNOW_RPM_UNAVAILABLE = N/A
  int16_t  heading;     // Rumbo en décimas de grado (0-3599)
  int16_t  altitude;    // Altitud en metros (-500 a 9000)
  uint8_t  speedSource; // ESPNOW_SOURCE_GPS | ESPNOW_SOURCE_OBD
  uint8_t  _reserved;   // Padding / futuro
};

// ==========================================================================
// Health — 1 Hz (30 bytes)
// ==========================================================================
struct __attribute__((packed)) EspNowHealth {
  EspNowHeader hdr;       // 8 bytes
  uint8_t  trackerState;  // ESPNOW_STATE_*
  uint8_t  gnssFix;       // 0=none, 2=2D, 3=3D
  uint8_t  gnssSats;      // Satélites visibles (0-99)
  uint8_t  gnssHdopX10;   // HDOP × 10 (0-250 → 0.0-25.0)
  uint8_t  lteSignal;     // CSQ (0-31, 99=unknown)
  uint8_t  lteRegistered; // 0=no, 1=sí
  uint8_t  mqttConnected; // 0=no, 1=sí
  uint8_t  mqttFailCount; // Fallos consecutivos MQTT (0-255)
  uint16_t batMv;         // Voltaje 18650 en mV (2000-4500)
  uint16_t vbusMv;        // Voltaje VBUS en mV (0 o 4000-6000)
  uint8_t  ignState;      // 0=UNKNOWN, 1=ON, 2=OFF
  uint8_t  wakeReason;    // esp_sleep_wakeup_cause_t
  uint16_t bootCount;     // rtcBootCount truncado a 16 bits
  uint16_t fixAgeS;       // Edad del fix GPS en segundos (0-65534, 65535=N/A)
  uint16_t gnssStale;     // Tramas rancias descartadas
  uint8_t  fwMajor;       // Versión firmware major
  uint8_t  fwMinor;       // Versión firmware minor
};

// ==========================================================================
// Event — envío inmediato (20 bytes)
// ==========================================================================
struct __attribute__((packed)) EspNowEvent {
  EspNowHeader hdr;       // 8 bytes
  uint8_t  eventType;     // ESPNOW_EVT_*
  uint8_t  ignState;      // Estado de ignición al momento
  uint16_t batMv;         // Voltaje batería al momento
  uint16_t vbusMv;        // Voltaje VBUS al momento
  uint16_t uptimeS;       // Segundos desde boot (truncado 16 bits)
  uint32_t _reserved;     // Futuro (OBD DTC, etc.)
};

// ==========================================================================
// Validación estática de tamaños
// ==========================================================================
// Si alguno de estos falla, el layout de memoria cambió y hay que actualizar
// ESPNOW_PROTOCOL_VERSION y el código de ambos lados.

#ifdef __cplusplus
static_assert(sizeof(EspNowHeader)    == 8,  "EspNowHeader debe ser 8 bytes");
static_assert(sizeof(EspNowFastTelem) == 18, "EspNowFastTelem debe ser 18 bytes");
static_assert(sizeof(EspNowHealth)    == 30, "EspNowHealth debe ser 30 bytes");
static_assert(sizeof(EspNowEvent)     == 20, "EspNowEvent debe ser 20 bytes");
static_assert(sizeof(float)           == 4,  "float debe ser 4 bytes (IEEE 754)");

// Validación explícita de la estructura heredada:
// En la documentación antigua se reportaba erróneamente como 18 bytes,
// pero su tamaño real packed siempre fue 23 bytes:
struct __attribute__((packed)) VehicleTelemetryLegacyCheck {
  uint8_t  magic;       // 1
  uint16_t seq;         // 2
  float    speed;       // 4
  uint16_t rpm;         // 2
  float    temp;        // 4
  float    voltage;     // 4
  uint8_t  satellites;  // 1
  uint8_t  fixType;     // 1
  int16_t  heading;     // 2
  int16_t  altitude;    // 2
};
static_assert(sizeof(VehicleTelemetryLegacyCheck) == 23, "VehicleTelemetryLegacyCheck debe ser 23 bytes");
#endif

// ==========================================================================
// CRC-8 (Dallas/Maxim, polinomio 0x8C reflejado)
// ==========================================================================
static inline uint8_t espnowCrc8(const uint8_t *data, size_t len) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; i++) {
    uint8_t b = data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      if ((crc ^ b) & 0x01)
        crc = (crc >> 1) ^ 0x8C;
      else
        crc >>= 1;
      b >>= 1;
    }
  }
  return crc;
}

// Calcula y escribe el CRC en el campo hdr.crc8 del paquete.
// El paquete debe tener hdr.crc8 = 0 antes de llamar, o esta función lo pone.
static inline void espnowSignPacket(void *pkt, size_t len) {
  uint8_t *raw = (uint8_t *)pkt;
  // Offset de crc8 dentro de EspNowHeader
  raw[offsetof(EspNowHeader, crc8)] = 0;
  raw[offsetof(EspNowHeader, crc8)] = espnowCrc8(raw, len);
}

// Verifica el CRC de un paquete recibido. Devuelve true si es válido.
static inline bool espnowVerifyPacket(const void *pkt, size_t len) {
  if (len < sizeof(EspNowHeader)) return false;
  uint8_t buf[250]; // ESP-NOW max payload
  if (len > sizeof(buf)) return false;
  memcpy(buf, pkt, len);
  uint8_t received = buf[offsetof(EspNowHeader, crc8)];
  buf[offsetof(EspNowHeader, crc8)] = 0;
  return espnowCrc8(buf, len) == received;
}

// ==========================================================================
// Validación de rangos (receptor)
// ==========================================================================
static inline bool espnowValidateFastTelem(const EspNowFastTelem *p) {
  if (p->speedX10 < 0 || p->speedX10 > 2200) return false;
  if (p->rpm != ESPNOW_RPM_UNAVAILABLE && p->rpm > 8000) return false;
  if (p->heading < 0 || p->heading > 3599) return false;
  if (p->altitude < -500 || p->altitude > 9000) return false;
  if (p->speedSource > ESPNOW_SOURCE_INTERNAL) return false;
  return true;
}

static inline bool espnowValidateHealth(const EspNowHealth *p) {
  if (p->trackerState > ESPNOW_STATE_CUTOFF) return false;
  if (p->gnssFix != 0 && p->gnssFix != 2 && p->gnssFix != 3) return false;
  if (p->gnssSats > 99) return false;
  if (p->ignState > 2) return false;
  return true;
}

static inline bool espnowValidateEvent(const EspNowEvent *p) {
  if (p->eventType == 0 || p->eventType > ESPNOW_EVT_PARKED_SLEEP) return false;
  if (p->ignState > 2) return false;
  return true;
}

// Validación general de header (primera línea de defensa del receptor)
static inline bool espnowValidateHeader(const EspNowHeader *h, uint8_t receivedLen) {
  if (h->version != ESPNOW_PROTOCOL_VERSION) return false;
  if (h->length != receivedLen) return false;
  if (h->msgType < ESPNOW_MSG_FAST_TELEM || h->msgType > ESPNOW_MSG_EVENT) return false;
  return true;
}
