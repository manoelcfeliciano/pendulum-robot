#include <Wire.h>
#include <Kalman.h>

void setup() {
  InitMotors();
  InitSensors();
}

void loop() {
  updateValues();
  verify_and_drive();
  delay(2);
}
