#ifndef IMU_TESTING_H_
#define IMU_TESTING_H_

#include <Arduino.h>

class IMU_Testing {
  public:
    IMU_Testing();
    printIMUData(void);
    checkOrient(void);

  private: 
    const signed char orientationMatrix;

};

#endif