#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println();
  Serial.println("=================================");
  Serial.println("GPS Tracker Logan - Board Check");
  Serial.println("=================================");
  Serial.println("R1: ESP32-S3 firmware is running.");
  Serial.println("USB CDC serial is working.");
  Serial.print("Millis at boot: ");
  Serial.println(millis());
}

void loop() {
  Serial.print("Heartbeat ms=");
  Serial.println(millis());
  delay(1000);
}