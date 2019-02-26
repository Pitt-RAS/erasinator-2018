#ifndef IMU_H_
#define IMU_H_

#include <Arduino.h>
#include <SparkFunMPU9250-DMP.h>
#define SerialPort SerialUSB

class IMU {
  public:
    IMU();
    float getAccelX();
    float getAccelY();
    float getAccelZ();
    float calcAngleFromX(bool);
    float calcAngleFromY(bool);

  private: 
    static const signed char orientationMatrix[];
    static unsigned char lastOrient;
    
    void setupOrient();
    void setupGyro();
    void setupAccel();
    void printOrientData();
    void printGyroData();
    void printAccelData();
    float getQX();
    float getQY();
    float getQZ();
    float getRollRad();
    float getPitchRad();
    float getYawRad();
    float getRollDeg();
    float getPitchDeg();
    float getYawDeg();
    float getGyroX();
    float getGyroY();
    float getGyroZ();
    
    void checkOrient();
    void checkGyro();
    void checkAccel();

};

#endif
