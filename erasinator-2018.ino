#include "EdgeDetector.h"
#include "MotorController.h"
#include "Odometry.h"
#include <PID_v1.h>
#include "math.h"

Odometry odometry;

long current_time = 0;
long last_turn = 0;
long previous_time_heart = -999;
long previous_time_update = -999;
uint8_t led_output = LOW;


double goalA, goalB, readingA, readingB, settingA, settingB;
double readings[2];

PID PIDA(&readingA, &settingA, &goalA, 3, .1, 0, DIRECT);
PID PIDB(&readingB, &settingB, &goalB, 3, .1, 0, DIRECT);

double courseHeading; //Desired Heading
double degreesOff,abDelta; //Difference from desired heading, difference in wheel velocities

PID PIDDelta(&degreesOff, &abDelta, 0, 4.5, 1, .1, DIRECT);

double courseVelocity, currentVelocity, abAverage;
PID PIDVelocity(&currentVelocity, &abAverage, &courseVelocity, 1, 0, 0, DIRECT);


MotorController motors;
double PWMA, PWMB;
void setup() {
// put your setup code here, to run once:
Serial.begin(9600);
pinMode(13, OUTPUT);
readingA = 0;
readingB = 0;
settingA = 0;
settingB = 0;
goalA = 30;
goalB = 30;
courseHeading=0;
courseVelocity=0;
PIDA.SetOutputLimits(-255.0, 255.0);
PIDB.SetOutputLimits(-255.0, 255.0);
PIDDelta.SetOutputLimits(-100, 100);
PIDDelta.SetMode(MANUAL);
PIDA.SetMode(AUTOMATIC);
PIDB.SetMode(AUTOMATIC);
//motors.moveA(false, 255);
//motors.moveB(false, 255);
PIDVelocity.SetMode(MANUAL);
abAverage=100;

}

void loop() {
if((current_time - previous_time_update) > 50) {
odometry.update();
previous_time_update = current_time;
odometry.getVelocity(readings);
readingA = readings[0];
readingB = readings[1];
degreesOff = courseHeading - odometry.getHeadingDegrees();
if (isnan(readingA)){
readingA=0;
readingB=0;
}
}
if (PIDDelta.Compute()){
goalA=abAverage+abDelta;
goalB=abAverage-abDelta;
}
if (PIDA.Compute()){
PWMA = settingA ;//+ 20*sin(odometry.getHeading());
if(PWMA<0){
motors.moveA(true, -PWMA);
}
else{
motors.moveA(false, PWMA);
}
}
if (PIDB.Compute()){
PWMB = settingB ;//+ //20*sin(odometry.getHeading());
if(PWMB<0){
motors.moveB(true, -PWMB);
}
else{
motors.moveB(false, PWMB);
}
}
//if(current_time-last_turn>10000){
//  courseHeading=-90;
//}
// if(current_time-last_turn>20000){
//  courseHeading=0;
//  last_turn=current_time;
//}

// Heartbeat stuff
if ((current_time - previous_time_heart) > 500) {
//Serial.printf("%.2f,%.2f, %.2f, %.2f, %.2f\n", readings[0], readings[1], settingA, settingB,odometry.getHeadingDegrees());
Serial.printf("%.2f,%.2f,%.2f,%.2f\n", abDelta,settingA, settingB, odometry.getHeadingDegrees());
}
current_time = millis();
}
