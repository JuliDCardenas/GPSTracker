#include <Arduino.h>

#define MODEM_RX 5   // ESP32 recibe desde TX del modem
#define MODEM_TX 4   // ESP32 envía hacia RX del modem
#define MODEM_PWRKEY 46

HardwareSerial SerialAT(1);

void modemPowerOn() {
  pinMode(MODEM_PWRKEY, OUTPUT);
  digitalWrite(MODEM_PWRKEY, HIGH);
  delay(100);
  // Pulso típico PWRKEY: mantener LOW ~1s y soltar (depende del módulo)
  digitalWrite(MODEM_PWRKEY, LOW);
  delay(1200);
  digitalWrite(MODEM_PWRKEY, HIGH);
  delay(3000);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\n[BOOT] AT Passthrough starting...");
  modemPowerOn();

  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  Serial.println("[BOOT] Type AT commands here. Expect OK.");
}

void loop() {
  // PC -> Modem
  while (Serial.available()) {
    SerialAT.write(Serial.read());
  }
  // Modem -> PC
  while (SerialAT.available()) {
    Serial.write(SerialAT.read());
  }
}