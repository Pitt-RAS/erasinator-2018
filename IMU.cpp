#include "IMU.h"

IMU::IMU() {
    // Call imu.begin() to verify communication and initialize
    if (imu.begin() != INV_SUCCESS) {
        while (1) {
            delay(5000);
        }
    }
    imu.setSensors(INV_XYZ_ACCEL); //Enable Gyro and Accel
    imu.setAccelFSR(2);
    imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL, 10);
}

//Calculates the angle from the current gravity vector away from x
//Outputs radians or degrees depending
float IMU::calcAngleFromX(bool degrees) {
    imu.dmpUpdateFifo();
    double mag = sqrt(imu.ax * imu.ax + imu.ay * imu.ay + imu.az * imu.az);
    float x = -1 * imu.ax;
    float calc = x / mag;
    float angle = acos(calc);
    if (imu.ay < 0) {
        angle *= -1.0;
    }
    if (degrees) {
        angle *= (180.0 / PI);
        if (angle < 0) {
             angle = 360.0 + angle;
        }
        return angle;
    }
    if (angle < 0) {
        angle = 2 * PI + angle;
    }
    return angle;
}

//Calculates the angle from the current gravity vector away from y
//Outputs radians or degrees depending
float IMU::calcAngleFromY(bool degrees) {
    imu.dmpUpdateFifo();
    double mag = sqrt(imu.ax * imu.ax + imu.ay * imu.ay + imu.az * imu.az);
    float y = imu.ay;
    float calc = y / mag;
    float angle = -acos(calc);
    if (imu.ax < 0) {
        angle *= -1.0;
    }
    if (degrees) {
        angle *= (180.0 / PI);
        if (angle < 0) {
            angle = 360.0 + angle;
        }
        return angle;
    }
    if (angle < 0) {
        angle = 2 * PI + angle;
    }
    return angle;
}
