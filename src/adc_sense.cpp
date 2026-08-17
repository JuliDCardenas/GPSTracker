// PRUEBA AISLADA: divisor de tension de ignicion (VBUS post-buck) -> GPIO9
//
// Hardware (baquelita, validado en papel 2026-08-15):
//   VBUS --[47k]--+--[510]--> GPIO9 (ADC1_CH8)
//                 |
//               [68k]
//                 |
//   GND ----------+
//   + 100nF en paralelo con R2 (68k) y Zener 3.6V (catodo hacia el pin)
//
//   VBUS = V_pin x 1.6912   (ej: 5.00V VBUS -> 2.96V en el pin)
//
// Umbrales documentados (medidos EN EL PIN):
//   IGN ON  : V_pin > 2.5V sostenido 2-3 s
//   IGN OFF : V_pin < 0.6V sostenido 20-30 s (absorbe el crank)
//
// Este firmware SOLO lee el ADC y reporta por serial. No inicializa el
// modem ni ningun otro periferico. Alimentar la LilyGO por USB.
// NO conectar los 12V del carro: la prueba de banco es con 5V/0V sobre
// la entrada VBUS del divisor (ej. salida del buck o un USB).

#include <Arduino.h>

#define SENSE_PIN 9  // GPIO9 = ADC1_CH8

static const float DIVIDER_FACTOR = 1.6912f;  // VBUS = V_pin * factor
static const float PIN_ON_V  = 2.5f;          // umbral ON medido en el pin
static const float PIN_OFF_V = 0.6f;          // umbral OFF medido en el pin

static const uint32_t REPORT_PERIOD_MS = 250;
static const int SAMPLES = 32;                // promedio para reducir ruido del ADC

void setup() {
  Serial.begin(115200);
  delay(500);

  analogReadResolution(12);
  // Atenuacion 12dB -> rango util ~0-3.1V (cubre los 2.96V del divisor).
  // Si compilas con core Arduino 2.x, cambiar a ADC_11db.
  analogSetPinAttenuation(SENSE_PIN, ADC_12db);

  Serial.println();
  Serial.println("[ADC_SENSE] GPIO9 (ADC1_CH8) - prueba divisor de ignicion");
  Serial.println("[ADC_SENSE] raw | V pin | VBUS estimado | estado");
}

void loop() {
  uint32_t accRaw = 0;
  uint32_t accMv  = 0;
  for (int i = 0; i < SAMPLES; i++) {
    accRaw += analogRead(SENSE_PIN);
    accMv  += analogReadMilliVolts(SENSE_PIN);
    delay(2);
  }

  float raw  = accRaw / (float)SAMPLES;
  float vPin = (accMv / (float)SAMPLES) / 1000.0f;
  float vbus = vPin * DIVIDER_FACTOR;

  const char *estado = vPin > PIN_ON_V  ? "IGN ON"  :
                       vPin < PIN_OFF_V ? "IGN OFF" : "ZONA MEDIA";

  Serial.printf("raw=%4.0f | pin=%.3fV | VBUS=%.2fV | %s\n", raw, vPin, vbus, estado);

  delay(REPORT_PERIOD_MS);
}
