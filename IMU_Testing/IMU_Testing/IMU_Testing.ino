#include <SparkFunMPU9250-DMP.h>
#define SerialPort SerialUSB

MPU9250_DMP imu;

unsigned long stepCount = 0;
unsigned long stepTime = 0;
unsigned long lastStepCount = 0;

const signed char orientationMatrix[9] = {
  1, 0, 0,
  0, 1, 0,
  0, 0, 1
};
unsigned char lastOrient = 0;

void setup() 
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
  
//  imu.dmpBegin(DMP_FEATURE_ANDROID_ORIENT);
  imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL | DMP_FEATURE_ANDROID_ORIENT, 10);
  imu.dmpSetOrientation(orientationMatrix);
}

void printIMUData(void)
{  
  // After calling dmpUpdateFifo() the ax, gx, mx, etc. values
  // are all updated.
  // Quaternion values are, by default, stored in Q30 long
  // format. calcQuat turns them into a float between -1 and 1
  float q0 = imu.calcQuat(imu.qw);
  float q1 = imu.calcQuat(imu.qx);
  float q2 = imu.calcQuat(imu.qy);
  float q3 = imu.calcQuat(imu.qz);

  SerialPort.println("Q: " + String(q0, 4) + ", " +
                    String(q1, 4) + ", " + String(q2, 4) + 
                    ", " + String(q3, 4));
  SerialPort.println("R/P/Y: " + String(imu.roll) + ", "
            + String(imu.pitch) + ", " + String(imu.yaw));
  SerialPort.println("Time: " + String(imu.time) + " ms");
  SerialPort.println();
}

void loop() 
{
  if ( imu.fifoAvailable() )
  {
    imu.dmpUpdateFifo();
    unsigned char orient = imu.dmpGetOrientation();
    if (orient != lastOrient)
    {
      switch (orient)
      {
      case ORIENT_PORTRAIT:
        SerialPort.println("Going up");
        imu.computeEulerAngles();
        printIMUData();
        break;
      case ORIENT_LANDSCAPE:
        SerialPort.println("Going left");
        imu.computeEulerAngles();
        printIMUData();
        break;
      case ORIENT_REVERSE_PORTRAIT:
        SerialPort.println("Going down");
        imu.computeEulerAngles();
        printIMUData();
        break;
      case ORIENT_REVERSE_LANDSCAPE:
        SerialPort.println("Going right");
        imu.computeEulerAngles();
        printIMUData();
        break;
      }
      lastOrient = orient;
    }
    
  }
}
