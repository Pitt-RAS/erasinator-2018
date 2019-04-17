#include "MotorPID.h"
#include <PID_v1.h>


MotorPID::MotorPID() {
  readingA = 0;
  readingB = 0;
  settingA = 0;
  settingB = 0;
  goalA = 0;
  goalB = 0;
  courseHeading = 0;
  courseVelocity = 0;
  cyclesSteady = 0;
  //PIDA = PID(&readingA, &settingA, &goalA, 2, .75, .1, DIRECT);
  //PIDB = PID(&readingB, &settingB, &goalB, 2, .75, .1, DIRECT);
  //PIDDelta = PID(&degreesOff, &abDelta, 0, 4, .5, .05, DIRECT);
  //PIDVelocity = PID(&currentVelocity, &abAverage, &courseVelocity, 1, 0, 0, DIRECT);
  PIDA.SetOutputLimits(-255.0, 255.0);
  PIDB.SetOutputLimits(-255.0, 255.0);
  PIDDelta.SetOutputLimits(-100, 100);
  PIDDistance.SetOutputLimits(-100.0,100.0);
  PIDDelta.SetMode(AUTOMATIC);
  PIDA.SetMode(AUTOMATIC);
  PIDB.SetMode(AUTOMATIC);
  //motors.moveA(false, 255);
  //motors.moveB(false, 255);
  PIDVelocity.SetMode(AUTOMATIC);
  PIDDistance.SetMode(MANUAL);
  abAverage = 0;
}
void MotorPID::update() {
  //Serial.printf("%.2f  //   %.2f",currentDistance, courseDistance);
  if ((current_time - previous_time_update) > 50) {
    odometry.update();
    previous_time_update = current_time;
    odometry.getVelocity(readings);
    readingA = readings[0];
    readingB = readings[1];
    degreesOff = courseHeading - odometry.getHeadingDegrees();
    currentDistance = odometry.getDistanceTraveled();
  }
  
  //Serial.printf("\t\t%.2f  //   %.2f",currentDistance, courseDistance);
  //Serial.printf("%.2f \t\t %.2f",readingA,readingB);
  //Serial.printf("%.2f", odometry.getHeadingDegrees());
 
  if(VMode){
    PIDVelocity.Compute();
  }
  else{
    
    PIDDistance.Compute();
  }
  if (PIDDelta.Compute()) {
    goalA = abAverage + abDelta;
    goalB = abAverage - abDelta;
  }
  //!
  //goalA = 20;
  //goalB = 20;
  //!
  if (PIDA.Compute()) {
    PWMA = settingA ;//+ 20*sin(odometry.getHeading());
    if(!paused){
      if (PWMA < 0) {
        motors.moveA(false, -PWMA);
      }
      else {
        motors.moveA(true, PWMA);
      }
    }
    else{
        motors.moveA(true, 0);
      
    }
    
  }
  if (PIDB.Compute()) {
    PWMB = settingB ;//+ //20*sin(odometry.getHeading());
    if(!paused){
      if (PWMB < 0) {
        motors.moveB(false, -PWMB);
      }
      else {
        motors.moveB(true, PWMB);
      }
    }
    else{
        motors.moveA(true, 0);
      
    }
  }
  if(VMode){
    if(abs(degreesOff)<5){
      //steady = true;
      cyclesSteady = min(20,cyclesSteady+1);
    }
    else{
      //steady = false;
      cyclesSteady = 0;
    }
  }else{
    if(abs(degreesOff)<5 && abs(courseDistance-currentDistance)<40 && abs(readingA)<5 && abs(readingB)<5){
      //steady=true;
      cyclesSteady = min(20,cyclesSteady+1);
    }
    else{
      //steady= false;
      cyclesSteady = 0;
    }
  }
  if(cyclesSteady==20){
    steady = true;
  }
  else{
    steady = false;
  }

  //Serial.printf("\t%.2f \t %.2f\t %.2f\t %.2f \t\t%d ",degreesOff,courseDistance-currentDistance,readingA, readingB, steady);
  current_time = millis();
}
void MotorPID::setVelocity(double v){
  steady = false;
  cyclesSteady = 0;
  courseVelocity = v;
}
void MotorPID::setHeading(double h){
  steady = false;
  cyclesSteady = 0;
  courseHeading = h;
}
void MotorPID::setMode(bool m){
  steady = false;
  cyclesSteady = 0;
  if(m){
    PIDVelocity.SetMode(AUTOMATIC);
    PIDDistance.SetMode(MANUAL);
  }
  else{
    PIDVelocity.SetMode(MANUAL);
    PIDDistance.SetMode(AUTOMATIC);
  }
  VMode = m;
}
void MotorPID::pause(bool p){
  paused = p;
}
bool MotorPID::still(){
  if(abs(settingA )<20 && abs(settingB )<20){
    return true;
  }
  else{
    return false;
  }
}
bool MotorPID::getSteady(){
  return steady;
}
void MotorPID::resetDistance(){
  odometry.zeroDistance();
  currentDistance = 0;
}
double MotorPID::getDistance(){
  return currentDistance;
}
void MotorPID::setDistance(double d){
  
  courseDistance = d;
}
