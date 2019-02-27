#ifndef IMU_H_
#define IMU_H_

#include <Arduino.h>
#include <SparkFunMPU9250-DMP.h>

class IMU {
  public:
    IMU();
    float calcAngleFromX(bool);
    float calcAngleFromY(bool);

  private:
    MPU9250_DMP imu;
};

#endif
