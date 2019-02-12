#include <Arduino.h>
#include "IMU_Testing.h"

IMU_Testing testing;

void setup() {
  testing.setupGyro();
  //testing.setupOrient();
}

void loop() {
  testing.checkGyro();
  //testing.checkOrient();
}
