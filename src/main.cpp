#include <Arduino.h>

#define MODEM_RX 4   // RX del ESP (viene del TX del módem)
#define MODEM_TX 5   // TX del ESP (va al RX del módem)

void setup() {
  Serial.begin(115200);
  delay(1200);
  Serial.println("\nBoot OK");

  Serial1.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(300);

  Serial.println("Sending AT...");
  Serial1.println("AT");
}

void loop() {
  while (Serial1.available()) {
    Serial.write(Serial1.read());
  }
  delay(10);
}