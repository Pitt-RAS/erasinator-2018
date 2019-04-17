#ifndef MOTOR_PID_H_
#define MOTOR_PID_H_

#include <Arduino.h>
#include "MotorController.h"
#include "Odometry.h"
#include <PID_v1.h>
#include "math.h"

class MotorPID {
  public:
    MotorPID();
    void update();
    void setVelocity(double);
    void setHeading(double);
    void setDistance(double);
    void setMode(bool);
    void pause(bool);
    bool still();
    bool getSteady();
    void resetDistance();
    double getDistance();
    Odometry odometry;
  private: 
    double goalA, goalB, readingA, readingB, settingA, settingB;
    double readings[2];
    PID PIDA = PID(&readingA, &settingA, &goalA, 2, .75, .1, DIRECT);
    PID PIDB = PID(&readingB, &settingB, &goalB, 2, .75, .1, DIRECT);
    double courseHeading; //Desired Heading
    double degreesOff,abDelta; //Difference from desired heading, difference in wheel velocities
    PID PIDDelta = PID(&degreesOff, &abDelta, 0, 4, .5, .05, DIRECT);
    bool VMode = true;
    double courseVelocity, currentVelocity, abAverage;
    PID PIDVelocity = PID(&currentVelocity, &abAverage, &courseVelocity, 1, 0, 0, DIRECT);
    double courseDistance, currentDistance;
    PID PIDDistance = PID(&currentDistance, &abAverage, &courseDistance, 1.5, .01, 0, DIRECT);
    MotorController motors;
    double PWMA, PWMB;
    long current_time = 0;
    long previous_time_update = -999;
    bool paused = false;
    bool steady = true;
    int cyclesSteady;
};

#endif
