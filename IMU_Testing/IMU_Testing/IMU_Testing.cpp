#include "IMU_Testing.h"
#include <SparkFunMPU9250-DMP.h>
#define SerialPort SerialUSB

MPU9250_DMP imu;

unsigned long IMU_Testing::stepCount = 0;
unsigned long IMU_Testing::stepTime = 0;
unsigned long IMU_Testing::lastStepCount = 0;

const signed char IMU_Testing::orientationMatrix[9]={1,0,0,0,1,0,0,0,1};
unsigned char IMU_Testing::lastOrient = 0;

<<<<<<< HEAD
//Initializes everything and gets the imu running
IMU_Testing::IMU_Testing()
=======
IMU_Testing::IMU_Testing()
{}

//Initializes everything and gets the imu running for Orientation and Quatorion 
void IMU_Testing::setupOrient()
>>>>>>> 5602601... Realized I made a minor error in the getters
{
    SerialPort.begin(115200);

    // Call imu.begin() to verify communication and initialize
    if (imu.begin() != INV_SUCCESS)
    {
        while (1){
          SerialPort.println("Unable to communicate with MPU-9250");
          SerialPort.println("Check connections, and try again.");
          SerialPort.println();
          delay(5000);
        }
    }
  
//  imu.dmpBegin(DMP_FEATURE_ANDROID_ORIENT);
    imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL | DMP_FEATURE_ANDROID_ORIENT, 10);
    imu.dmpSetOrientation(orientationMatrix);
}

<<<<<<< HEAD
//Prints the Roll, Pitch, and Yaw of the robot in relation to the sensor
void IMU_Testing::printIMUData()
=======
//Initializes everything and gets the imu running for Gyro
void IMU_Testing::setupGyro()
{
  SerialPort.begin(115200);

  // Call imu.begin() to verify communication and initialize
  if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      SerialPort.println("Unable to communicate with MPU-9250");
      SerialPort.println("Check connections, and try again.");
      SerialPort.println();
      delay(5000);
    }
  }

  imu.setSensors(INV_XYZ_GYRO); // Enable gyroscope only
  imu.setGyroFSR(2000); // Set gyro to 2000 dps

  imu.dmpBegin(DMP_FEATURE_GYRO_CAL |   // Enable gyro cal
              DMP_FEATURE_SEND_CAL_GYRO,// Send cal'd gyro values
              10);                   // Set DMP rate to 10 Hz
}

void IMU_Testing::setupAccel()
{
   SerialPort.begin(115200);

    // Call imu.begin() to verify communication and initialize
    if (imu.begin() != INV_SUCCESS)
    {
        while (1){
          SerialPort.println("Unable to communicate with MPU-9250");
          SerialPort.println("Check connections, and try again.");
          SerialPort.println();
          delay(5000);
        }
    }

    imu.setSensors(INV_XYZ_ACCEL);
    imu.setAccelFSR(16);

    imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL, 10);
}

//Prints the Roll, Pitch, and Yaw of the robot in relation to the sensor
void IMU_Testing::printOrientData()
>>>>>>> 5602601... Realized I made a minor error in the getters
{  
  // After calling dmpUpdateFifo() the ax, gx, mx, etc. values
  // are all updated.
  // Quaternion values are, by default, stored in Q30 long
  // format. calcQuat turns them into a float between -1 and 1
  float q0 = imu.calcQuat(imu.qw);
  float q1 = imu.calcQuat(imu.qx);
  float q2 = imu.calcQuat(imu.qy);
  float q3 = imu.calcQuat(imu.qz);

  SerialPort.println("Q: " + String(q0, 4) + ", " + String(q1, 4) + ", " + String(q2, 4) + 
                    ", " + String(q3, 4));
  SerialPort.println("R/P/Y: " + String(imu.roll) + ", " + String(imu.pitch) + ", " + String(imu.yaw));
  SerialPort.println("Time: " + String(imu.time) + " ms");
  SerialPort.println();
}

<<<<<<< HEAD
/*Checks to see if the orientation was changed and prints out the change along with
the data from printIMUData();*/
void IMU_Testing::checkOrient(){
=======
//Prints the x, y, and z axis of the robot in relation to the sensor
void IMU_Testing::printGyroData()
{
  // After calling dmpUpdateFifo() the ax, gx, mx, etc. values
  // are all updated.
  float gyroX = imu.calcGyro(imu.gx);
  float gyroY = imu.calcGyro(imu.gy);
  float gyroZ = imu.calcGyro(imu.gz);
  
  SerialPort.println("Gyro: " + String(gyroX) + ", " +
              String(gyroY) + ", " + String(gyroZ) + " dps");
  SerialPort.println("Time: " + String(imu.time) + " ms");
  SerialPort.println();
}

//Prints the x, y, and z axis acceleration of the robot in relation to the sensor
void IMU_Testing::printAccelData()
{
  float accelX = imu.calcAccel(imu.ax);
  float accelY = imu.calcAccel(imu.ay);
  float accelZ = imu.calcAccel(imu.az);

  SerialPort.println("Accel: " + String(accelX) + ", " +
              String(accelY) + ", " + String(accelZ) + " dps");
  SerialPort.println("Time: " + String(imu.time) + " ms");
  SerialPort.println();
}

//Outputs the raw calculation of the x axis Quaternion
float IMU_Testing::getQX()
{
  imu.dmpUpdateFifo();
  return imu.calcQuat(imu.qx);
}

//Outputs the raw calculation of the y axis Quaternion
float IMU_Testing::getQY()
{
  imu.dmpUpdateFifo();
  return imu.calcQuat(imu.qy);
}

//Outputs the raw calculation of the z axis Quaternion
float IMU_Testing::getQZ()
{
  imu.dmpUpdateFifo();
  return imu.calcQuat(imu.qz);
}

//Outputs the raw calculation of the x axis Gyro
float IMU_Testing::getGyroX()
{
  imu.dmpUpdateFifo();
  return imu.calcGyro(imu.gx);
}

//Outputs the raw calculation of the y axis Gyro
float IMU_Testing::getGyroY()
{
  imu.dmpUpdateFifo();
  return imu.calcGyro(imu.gy);
}

//Outputs the raw calculation of the z axis Gyro
float IMU_Testing::getGyroZ()
{
  imu.dmpUpdateFifo();
  return imu.calcGyro(imu.gz);
}

//Outputs the raw calculation of the x axis Acceleration
float IMU_Testing::getAccelX()
{
  imu.dmpUpdateFifo();
  return imu.calcAccel(imu.ax);
}

//Outputs the raw calculation of the y axis Acceleration
float IMU_Testing::getAccelY()
{
  imu.dmpUpdateFifo();
  return imu.calcAccel(imu.ay);
}

//Outputs the raw calculation of the z axis Acceleration
float IMU_Testing::getAccelZ()
{
  imu.dmpUpdateFifo();
  return imu.calcAccel(imu.az);
}

/*Checks to see if the orientation was changed and prints out the change along with
the data from printIMUData();*/
void IMU_Testing::checkOrient()
{
>>>>>>> 5602601... Realized I made a minor error in the getters
    if ( imu.fifoAvailable() ){
        imu.dmpUpdateFifo();
        unsigned char orient = imu.dmpGetOrientation();
        if (orient != lastOrient){
            switch (orient){
                case ORIENT_PORTRAIT:
                    SerialPort.println("Going up");
                    imu.computeEulerAngles();
<<<<<<< HEAD
                    printIMUData();
                    //do something
=======
                    printOrientData();
                      //do something
>>>>>>> 5602601... Realized I made a minor error in the getters
                    break;
                case ORIENT_LANDSCAPE:
                    SerialPort.println("Going left");
                    imu.computeEulerAngles();
<<<<<<< HEAD
                    printIMUData();
                    //do something
=======
                    printOrientData();
                      //do something
>>>>>>> 5602601... Realized I made a minor error in the getters
                    break;
                case ORIENT_REVERSE_PORTRAIT:
                    SerialPort.println("Going down");
                    imu.computeEulerAngles();
<<<<<<< HEAD
                    printIMUData();
                    //do something
=======
                    printOrientData();
                      //do something
>>>>>>> 5602601... Realized I made a minor error in the getters
                    break;
                case ORIENT_REVERSE_LANDSCAPE:
                    SerialPort.println("Going right");
                    imu.computeEulerAngles();
<<<<<<< HEAD
                    printIMUData();
                    //do something
=======
                    printOrientData();
                      //do something
>>>>>>> 5602601... Realized I made a minor error in the getters
                    break;
            }
            lastOrient = orient;
        }
    }
}
<<<<<<< HEAD
=======

//Checks and updates the orientation of the robot according to the gyro
void IMU_Testing::checkGyro()
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
void IMU_Testing::checkAccel()
{
  if( imu.fifoAvailable() ){
    if( imu.dmpUpdateFifo() == INV_SUCCESS){
      printAccelData();
    }
  }
}
>>>>>>> 5602601... Realized I made a minor error in the getters
