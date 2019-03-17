#ifndef IMU_TESTING_H_
#define IMU_TESTING_H_

#include <Arduino.h>

class IMU_Testing {
  public:
    IMU_Testing();
    void setupOrient();
    void setupGyro();
    void setupAccel();
    void printOrientData();
    void printGyroData();
    void printAccelData();
    float getQX();
    float getQY();
    float getQZ();
    float getGyroX();
    float getGyroY();
    float getGyroZ();
    float getAccelX();
    float getAccelY();
    float getAccelZ();
    void checkOrient();
    void checkGyro();
    void checkAccel();

  private: 
    static unsigned long stepCount;
    static unsigned long stepTime;
    static unsigned long lastStepCount;
    
    static const signed char orientationMatrix[];
    static unsigned char lastOrient;

};

#endif
