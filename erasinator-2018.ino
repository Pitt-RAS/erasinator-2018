#include "MotorPID.h"

Odometry odometry;

MotorPID motorPid;
int stepI;
bool wasSteady = false;
bool started = false;
bool asdf = false;
void setup() {
  stepI = 0;
  delay(100);
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  motorPid.setMode(true);
  motorPid.setVelocity(0);
  motorPid.setHeading(0);
}

void loop() {
  motorPid.pause(false);
  motorPid.update();
  if((motorPid.getSteady() && wasSteady==false)||started==false){
    started = true;
    stepI++;
    motorPid.resetDistance();
    //Serial.printf("%d \n",stepI);
    if(stepI<10){
      if(stepI%9==0){// && stepI!=0){
        stepI++;
      }
      if(stepI%9==1){
        motorPid.setMode(false);
        motorPid.setHeading(0);
        motorPid.setDistance(500);
        delay(100);
      }
      if(stepI%9==2){
        motorPid.setMode(true);
        motorPid.setVelocity(0);
        motorPid.setHeading(-90);
        delay(100);
      }
      if(stepI%9==3){
        motorPid.setMode(false);
        motorPid.setDistance(100);
        delay(100);
      }
      if(stepI%9==4){
        motorPid.setMode(true);
        motorPid.setVelocity(0);
        motorPid.setHeading(-180);
        delay(100); 
      }
      if(stepI%9==5){
        motorPid.setMode(false);
        motorPid.setDistance(500);
        delay(100);
      }
      if(stepI%9==6){
        motorPid.setMode(true);
        motorPid.setVelocity(0);
        motorPid.setHeading(-90);
        delay(100);
      }
      if(stepI%9==7){
        motorPid.setMode(false);
        motorPid.setDistance(100);
        delay(100);
      }
      if(stepI%9==8){
        motorPid.setMode(true);
        motorPid.setVelocity(0);
        motorPid.setHeading(0);
        delay(100);
      }
    }
  }
  Serial.print("\n");
  wasSteady=motorPid.getSteady();
}
