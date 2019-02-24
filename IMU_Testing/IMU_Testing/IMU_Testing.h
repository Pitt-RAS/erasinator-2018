#ifndef IMU_TESTING_H_
#define IMU_TESTING_H_

#include <Arduino.h>

class IMU_Testing {
  public:
    IMU_Testing();
<<<<<<< HEAD
    void printIMUData();
    void checkOrient();
=======
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
>>>>>>> 5602601... Realized I made a minor error in the getters

  private: 
    static unsigned long stepCount;
    static unsigned long stepTime;
    static unsigned long lastStepCount;
    
    static const signed char orientationMatrix[];
    static unsigned char lastOrient;

};

#endif
