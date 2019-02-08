#include <Arduino.h>
#include "IMU_Testing.h"

IMU_Testing testing;

void setup() {}

void loop() {
  testing.checkOrient();
}
