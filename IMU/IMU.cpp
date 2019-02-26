#include "IMU.h"

MPU9250_DMP imu;

//orientationMatrix is used to set the initial orientation of the robot
//Basically an X, Y, and Z vector
const signed char IMU::orientationMatrix[9]={1,0,0,0,1,0,0,0,1};
unsigned char IMU::lastOrient = 0;

IMU::IMU()
{
  //  SerialPort.begin(115200);

  // Call imu.begin() to verify communication and initialize
  if (imu.begin() != INV_SUCCESS)
  {
    while (1){
//      SerialPort.println("Unable to communicate with MPU-9250");
//      SerialPort.println("Check connections, and try again.");
//      SerialPort.println();
      delay(5000);
    }
  }

  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL); //Enable Gyro and Accel
  imu.setGyroFSR(2000); // Set gyro to 2000 dps
  imu.setAccelFSR(2);
    
  imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | 
               DMP_FEATURE_GYRO_CAL | 
               DMP_FEATURE_ANDROID_ORIENT |
               DMP_FEATURE_SEND_CAL_GYRO |
               DMP_FEATURE_SEND_RAW_ACCEL
               , 10);
  imu.dmpSetOrientation(orientationMatrix);
}

//Initializes everything and gets the imu running for Orientation and Quatorion 
void IMU::setupOrient()
{
//    SerialPort.begin(115200);

    // Call imu.begin() to verify communication and initialize
    if (imu.begin() != INV_SUCCESS)
    {
        while (1){
//          SerialPort.println("Unable to communicate with MPU-9250");
//          SerialPort.println("Check connections, and try again.");
//          SerialPort.println();
          delay(5000);
        }
    }
  
    imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL | DMP_FEATURE_ANDROID_ORIENT, 10);
    imu.dmpSetOrientation(orientationMatrix);
}

//Initializes everything and gets the imu running for Gyro
void IMU::setupGyro()
{
//  SerialPort.begin(115200);

  // Call imu.begin() to verify communication and initialize
  if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
//      SerialPort.println("Unable to communicate with MPU-9250");
//      SerialPort.println("Check connections, and try again.");
//      SerialPort.println();
      delay(5000);
    }
  }

  imu.setSensors(INV_XYZ_GYRO); // Enable gyroscope only
  imu.setGyroFSR(2000); // Set gyro to 2000 dps

  imu.dmpBegin(DMP_FEATURE_GYRO_CAL |   // Enable gyro cal
              DMP_FEATURE_SEND_CAL_GYRO,// Send cal'd gyro values
              10);                   // Set DMP rate to 10 Hz
}

void IMU::setupAccel()
{
//   SerialPort.begin(115200);

    // Call imu.begin() to verify communication and initialize
    if (imu.begin() != INV_SUCCESS)
    {
        while (1){
//          SerialPort.println("Unable to communicate with MPU-9250");
//          SerialPort.println("Check connections, and try again.");
//          SerialPort.println();
          delay(5000);
        }
    }

    imu.setSensors(INV_XYZ_ACCEL);
    imu.setAccelFSR(2);

    imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL, 10);
}

//Prints the Roll, Pitch, and Yaw of the robot in relation to the sensor
void IMU::printOrientData()
{  
  // After calling dmpUpdateFifo() the ax, gx, mx, etc. values
  // are all updated.
  // Quaternion values are, by default, stored in Q30 long
  // format. calcQuat turns them into a float between -1 and 1
  float q0 = imu.calcQuat(imu.qw);
  float q1 = imu.calcQuat(imu.qx);
  float q2 = imu.calcQuat(imu.qy);
  float q3 = imu.calcQuat(imu.qz);

//  SerialPort.println("Q: " + String(q0, 4) + ", " + String(q1, 4) + ", " + String(q2, 4) + 
//                    ", " + String(q3, 4));
//  SerialPort.println("R/P/Y: " + String(imu.roll) + ", " + String(imu.pitch) + ", " + String(imu.yaw));
//  SerialPort.println("Time: " + String(imu.time) + " ms");
//  SerialPort.println();
}

//Prints the x, y, and z axis of the robot in relation to the sensor
void IMU::printGyroData()
{
  // After calling dmpUpdateFifo() the ax, gx, mx, etc. values
  // are all updated.
  float gyroX = imu.calcGyro(imu.gx);
  float gyroY = imu.calcGyro(imu.gy);
  float gyroZ = imu.calcGyro(imu.gz);
  
//  SerialPort.println("Gyro: " + String(gyroX) + ", " +
//              String(gyroY) + ", " + String(gyroZ) + " dps");
//  SerialPort.println("Time: " + String(imu.time) + " ms");
//  SerialPort.println();
}

//Prints the x, y, and z axis acceleration of the robot in relation to the sensor
void IMU::printAccelData()
{
  float accelX = imu.calcAccel(imu.ax);
  float accelY = imu.calcAccel(imu.ay);
  float accelZ = imu.calcAccel(imu.az);

//  SerialPort.println("Accel: " + String(accelX) + ", " +
//              String(accelY) + ", " + String(accelZ) + " dps");
//  SerialPort.println("Time: " + String(imu.time) + " ms");
//  SerialPort.println();
}

//Outputs the raw calculation of the x axis Quaternion
//Outputs Vector betwen +/- 1
float IMU::getQX()
{
  imu.dmpUpdateFifo();
  return imu.calcQuat(imu.qx);
}

//Outputs the raw calculation of the y axis Quaternion
//Outputs Vector betwen +/- 1
float IMU::getQY()
{
  imu.dmpUpdateFifo();
  return imu.calcQuat(imu.qy);
}

//Outputs the raw calculation of the z axis Quaternion
//Outputs Vector betwen +/- 1
float IMU::getQZ()
{
  imu.dmpUpdateFifo();
  return imu.calcQuat(imu.qz);
}

//Outputs the driect roll value calculated from Euler Angle (radians)
//If the robot were to piviot against the board along the y (forward/backwards) axis
float IMU::getRollRad()
{
  imu.dmpUpdateFifo();
  imu.computeEulerAngles(false);
  return imu.roll;
}

//Outputs the driect pitch value calculated from Euler Angle (radians)
//If the robot were to piviot against the board along the x (strafing) axis
float IMU::getPitchRad()
{
  imu.dmpUpdateFifo();
  imu.computeEulerAngles(false);
  return imu.pitch;
}

//Outputs the driect yaw value calculated from Euler Angle (radians)
//If the robot were to piviot against the board along the z (away from board) axis
//Useful for abstact values
float IMU::getYawRad()
{
  imu.dmpUpdateFifo();
  imu.computeEulerAngles(false);
  return imu.yaw;
}

//Outputs the driect roll value calculated from Euler Angle (degrees)
//If the robot were to piviot against the board along the y (forward/backwards) axis
float IMU::getRollDeg()
{
  imu.dmpUpdateFifo();
  imu.computeEulerAngles(true);
  return imu.roll;
}

//Outputs the driect pitch value calculated from Euler Angle (degrees)
//If the robot were to piviot against the board along the x (strafing) axis
float IMU::getPitchDeg()
{
  imu.dmpUpdateFifo();
  imu.computeEulerAngles(true);
  return imu.pitch;
}

//Outputs the driect yaw value calculated from Euler Angle (degrees)
//If the robot were to piviot against the board along the z (away from board) axis
//Most useful
float IMU::getYawDeg()
{
  imu.dmpUpdateFifo();
  imu.computeEulerAngles(true);
  return imu.yaw;
}

//Outputs the raw calculation of the x axis Gyro
//Value is to degree's per second
float IMU::getGyroX()
{
  imu.dmpUpdateFifo();
  return imu.calcGyro(imu.gx);
}

//Outputs the raw calculation of the y axis Gyro
//Value is to degree's per second
float IMU::getGyroY()
{
  imu.dmpUpdateFifo();
  return imu.calcGyro(imu.gy);
}

//Outputs the raw calculation of the z axis Gyro
//Value is to degree's per second
float IMU::getGyroZ()
{
  imu.dmpUpdateFifo();
  return imu.calcGyro(imu.gz);
}

//Outputs the raw calculation of the x axis Acceleration
//Strafing
//Value to g's
float IMU::getAccelX()
{
  imu.dmpUpdateFifo();
  return imu.calcAccel(imu.ax);
}

//Outputs the raw calculation of the y axis Acceleration
//Value to g's
//Forward
float IMU::getAccelY()
{
  imu.dmpUpdateFifo();
  return imu.calcAccel(imu.ay);
}

//Outputs the raw calculation of the z axis Acceleration
//Value to g's
//In and out of board
float IMU::getAccelZ()
{
  imu.dmpUpdateFifo();
  return imu.calcAccel(imu.az);
}

/*Checks to see if the orientation was changed and prints out the change along with
the data from printIMUData();*/
void IMU::checkOrient()
{
    if ( imu.fifoAvailable() ){
        imu.dmpUpdateFifo();
        unsigned char orient = imu.dmpGetOrientation();
        if (orient != lastOrient){
            switch (orient){
                case ORIENT_PORTRAIT:
//                    SerialPort.println("Going up");
                    imu.computeEulerAngles();
                    printOrientData();
                      //do something
                    break;
                case ORIENT_LANDSCAPE:
//                    SerialPort.println("Going left");
                    imu.computeEulerAngles();
                    printOrientData();
                      //do something
                    break;
                case ORIENT_REVERSE_PORTRAIT:
//                    SerialPort.println("Going down");
                    imu.computeEulerAngles();
                    printOrientData();
                      //do something
                    break;
                case ORIENT_REVERSE_LANDSCAPE:
//                    SerialPort.println("Going right");
                    imu.computeEulerAngles();
                    printOrientData();
                      //do something
                    break;
            }
            lastOrient = orient;
        }
    }
}

//Checks and updates the orientation of the robot according to the gyro
void IMU::checkGyro()
{
  // Check for new data in the FIFO
  if ( imu.fifoAvailable() )
  {
    // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
    if ( imu.dmpUpdateFifo() == INV_SUCCESS)
    {
      printGyroData();
    }
  }
}

//Checks and updates the acceperation of the robot according to the acceleration sensor
void IMU::checkAccel()
{
  if( imu.fifoAvailable() ){
    if( imu.dmpUpdateFifo() == INV_SUCCESS){
      printAccelData();
    }
  }
}

//Calculates the angle from the current gravity vector away from x
//Outputs radians or degrees depending 
float IMU::calcAngleFromX(bool degrees)
{
  imu.dmpUpdateFifo();
  double mag = sqrt(imu.ax * imu.ax + imu.ay * imu.ay + imu.az * imu.az);
  float x = -1 * imu.ax;
  float calc = x / mag;
  float angle = acos(calc);
  if(imu.ay < 0) angle *= -1.0;
  if(degrees){
    angle *= (180.0 / PI);
    if(angle < 0) angle = 360.0 + angle;
    return angle;
  }
  if(angle < 0){
    angle = 2*PI + angle;
  }
  return angle;
}

//Calculates the angle from the current gravity vector away from y
//Outputs radians or degrees depending 
float IMU::calcAngleFromY(bool degrees)
{
  imu.dmpUpdateFifo();
  double mag = sqrt(imu.ax * imu.ax + imu.ay * imu.ay + imu.az * imu.az);
  float y = imu.ay;
  float calc = y / mag;
  float angle = acos(calc);
  if(imu.ax < 0) angle *= -1.0;
  if(degrees){
    angle *= (180.0 / PI);
    if(angle < 0) angle = 360.0 + angle;
    return angle;
  }
  if(angle < 0){
    angle = 2*PI + angle;
  }
  return angle;
}
