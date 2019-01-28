/**
 * Odometry Code
 * Xinke Chen
 */ 
#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include <Encoder.h>
#include "Buffer.h"

#define RADS_DEGREE 180/M_PI;

class Odometry {
public:
    Odometry();
    double getHeading();
    double getHeadingDegrees();
    double getDistanceTraveled();
    double getVelocity();
    void update();
private:
    Encoder encoder_a_;
    Encoder encoder_b_;
    Buffer velocity_buffer_;
    Buffer heading_buffer_;
    void calculateDistanceTotal();
    void calculateVelocityInstantaneous(long delta_time, long delta_a, long delta_b);
    void calculateHeadingInstantaneous(long new_count_a, long new_count_b);
    double theta;
    double distance;
    double angular_velocity_a;
    double angular_velocity_b;
    double current_velocity;
    long previous_time;
    long last_motor_count_a;
    long last_motor_count_b;
    // Constants
    static const uint8_t ENCODE_A_1;
    static const uint8_t ENCODE_A_2;
    static const uint8_t ENCODE_B_1;
    static const uint8_t ENCODE_B_2;
    static const int GEAR_RATIO;
    static const int ENCODER_RESOLUTION;
    static const double WHEEL_RADIUS;
    static const double WHEEL_CIRCUMFERENCE; //in millimeters
    static const int COUNTS_PER_REVOLUTION;
    static const double WHEEL_TRACK;
};
#endif
