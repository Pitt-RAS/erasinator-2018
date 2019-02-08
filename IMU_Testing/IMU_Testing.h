#ifndef IMU_TESTING_H_
#define IMU_TESTING_H_

#include <Arduino.h>

class IMU_Testing {
  public:
    IMU_Testing();
    void printIMUData();
    void checkOrient();

  private: 
    static unsigned long stepCount;
    static unsigned long stepTime;
    static unsigned long lastStepCount;
    
    static const signed char orientationMatrix[];
    static unsigned char lastOrient;

};

#endif
