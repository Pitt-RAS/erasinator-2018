#include "EdgeDetector.h"
#include "MotorController.h"
#include "Odometry.h"


Odometry odometry;

long current_time = 0;
long previous_time_heart = -999;
long previous_time_update = -999;
uint8_t led_output = LOW;
double wheel_velocities[2];
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); 
  Serial.println("Starting");
  pinMode(13, OUTPUT);
}

void loop() {
  if((current_time - previous_time_update) > 50) {
    odometry.update();
    previous_time_update = current_time;
  }
  // Heartbeat stuff
  if ((current_time - previous_time_heart) > 500) {
    if (led_output == LOW) {
      led_output = HIGH;
      digitalWrite(13, led_output);
    } else {
      led_output = LOW;
      digitalWrite(13, led_output);
    }
    Serial.printf("Total Distance: %.2f\tHeading: %.2f\tVelocity: %0.2f\tA: %0.2f\t B: %0.2f\n\r", 
      odometry.getDistanceTraveled(), 
      odometry.getHeadingDegrees(),
      odometry.getVelocity(wheel_velocities),
      wheel_velocities[0],
      wheel_velocities[1]);
    previous_time_heart = current_time;
  }
  current_time = millis();
}
