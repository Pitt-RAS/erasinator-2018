#include <Arduino.h>
#include "IMU_Testing.h"

IMU_Testing testing;

<<<<<<< HEAD
void setup() {}

void loop() {
  testing.checkOrient();
=======
void setup() {
  testing.setupOrient();
  testing.setupGyro();
  testing.setupAccel();
}

void loop() {
  testing.checkOrient();
  testing.checkGyro();
  testing.checkAccel();
>>>>>>> 5602601... Realized I made a minor error in the getters
}
