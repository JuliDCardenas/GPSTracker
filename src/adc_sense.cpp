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
//   VBUS = V_pin x 1.9366   (factor CALIBRADO en banco 2026-08-17)
//
// CALIBRACION DEL DIVISOR (2026-08-17):
//   Medicion en banco, buck alimentado desde el carro:
//     salida del buck (multimetro) = 5.25V
//     V_pin (multimetro)           = 2.74V
//     V_pin (lectura del ADC)      = 2.711V
//   Relacion real = 2.74 / 5.25 = 0.5219 contra 0.5913 teorico de 47k/68k
//   => desviacion -11.7%. Causa probable: fuga inversa del Zener 3.6V
//   (equivalente ~210k / ~13uA) en paralelo con R2, sobre un divisor de muy
//   alta impedancia (~40uA de corriente de trabajo).
//   Factor empirico = 5.25 / 2.711 = 1.9366 (absorbe tambien el ~1% de error
//   del ADC frente al multimetro).
//   LIMITACION: al depender de una fuga, el VBUS reportado puede derivar con
//   la temperatura. La logica ON/OFF no se afecta porque los umbrales estan
//   definidos sobre el pin, no sobre VBUS.
//   PENDIENTE (hardware, tarea en Notion): bajar el divisor a 10k/12k y
//   ajustar el pot del MP1584 a 5.05-5.10V para recuperar una relacion
//   predecible; entonces recalibrar este factor.
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

// Factor calibrado 2026-08-17 (antes 1.6912f teorico). Ver nota de arriba.
static const float DIVIDER_FACTOR = 1.9366f;  // VBUS = V_pin * factor
static const float PIN_ON_V  = 2.5f;          // umbral ON medido en el pin
static const float PIN_OFF_V = 0.6f;          // umbral OFF medido en el pin

static const uint32_t REPORT_PERIOD_MS = 250;
static const int SAMPLES = 32;                // promedio para reducir ruido del ADC

void setup() {
  Serial.begin(115200);
  delay(500);

  analogReadResolution(12);
  // Atenuacion 11dB -> rango util ~0-3.1V (cubre los 2.96V del divisor).
  // (Core Arduino 3.x: el nombre equivalente es ADC_12db.)
  analogSetPinAttenuation(SENSE_PIN, ADC_11db);

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
