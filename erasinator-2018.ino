#include <PID_v1.h>
#include <Encoder.h>
#include "MotorController.h"
double rightSpeed, rightIn, rightOut, leftSpeed, leftIn, leftOut;
PID rightPID(&rightIn,&rightOut,&rightSpeed, 2, 5, 1, DIRECT);
PID leftPID(&leftIn,&leftOut,&leftSpeed, 2, 5, 1, DIRECT);
Encoder rightEncoder(5,6);
Encoder leftEncoder(7,8);
MotorController motors = MotorController();
void setup() {
  // put your setup code here, to run once:
  pinMode(7, INPUT);
  pinMode(8, INPUT);
  pinMode(5, INPUT);
  pinMode(6, INPUT);
  rightSpeed=1;
  leftSpeed=1;
  rightPID.SetMode(AUTOMATIC);
  leftPID.SetMode(AUTOMATIC);  
  Serial.begin(9600);
}

void loop() {
  rightIn=-1*rightEncoder.read();
  leftIn=-1*leftEncoder.read();
  rightPID.Compute();
  leftPID.Compute();
  Serial.println(rightIn);
  Serial.println(leftIn);
  Serial.println("--------");
  motors.moveA(true, rightOut);
  motors.moveB(true, leftOut);
  rightEncoder.write(0);
  leftEncoder.write(0);
  delay(10);
}
