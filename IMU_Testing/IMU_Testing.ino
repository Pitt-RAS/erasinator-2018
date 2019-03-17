#include <Arduino.h>
#include "IMU_Testing.h"

IMU_Testing testing;

void setup() {
  testing.setupOrient();
  testing.setupGyro();
  testing.setupAccel();
}

void loop() {
  testing.checkOrient();
  testing.checkGyro();
  testing.checkAccel();
}
