#include <Arduino.h>
#include "MotorController.h"

MotorController controller;

void setup() {}

void loop() {
  controller.moveA(true, 210);
  controller.moveB(true, 210);
}
