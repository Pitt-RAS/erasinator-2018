#include "MotorPID.h"

Odometry odometry;

MotorPID motorPid;
int stepI;
bool wasSteady = false;
void setup() {
  stepI = 0;
  delay(1000);
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  motorPid.setMode(true);
  motorPid.setVelocity(0);
  motorPid.setHeading(-90);
}

void loop() {
  
  motorPid.update();
  if(motorPid.getSteady() && wasSteady==false){
    stepI++;
    
  }
  if(stepI==1){
    motorPid.setMode(false);
    motorPid.setDistance(100);
  }
  if(stepI==2){
    motorPid.setMode(true);
    motorPid.setHeading(-180);
    
  }
  //Serial.printf("%d \n",stepI);

  wasSteady=motorPid.getSteady();
}
