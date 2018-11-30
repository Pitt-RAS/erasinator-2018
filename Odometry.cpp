#include "Odometry.h"

const uint8_t Odometry::ENCODE_A_1 = 5;
const uint8_t Odometry::ENCODE_A_2 = 6;
const uint8_t Odometry::ENCODE_B_1 = 7;
const uint8_t Odometry::ENCODE_B_2 = 8;
const int Odometry::GEAR_RATIO = 63;
const int Odometry::ENCODER_RESOLUTION = 20;
const double Odometry::WHEEL_RADIUS = 29.83;
const double Odometry::WHEEL_CIRCUMFERENCE = 187.301;
const double Odometry::WHEEL_TRACK = 192.0;
const int Odometry::COUNTS_PER_REVOLUTION = GEAR_RATIO * ENCODER_RESOLUTION;

Odometry::Odometry() :
    encoder_a_(ENCODE_A_1, ENCODE_A_2),
    encoder_b_(ENCODE_B_1, ENCODE_B_2) {
    theta = 0;
    distance = 0;
    previous_time = 0;
    angular_velocity_a = 0;
    angular_velocity_b = 0;
    current_velocity = 0;
    last_motor_count_a = -999;
    last_motor_count_b = -999;
    
}

void Odometry::update() {
    long new_motor_count_a = encoder_a_.read();
    long new_motor_count_b = encoder_b_.read();
    long diff_count_a = new_motor_count_a - last_motor_count_a;
    long diff_count_b = new_motor_count_b - last_motor_count_b;
    long current_time = millis();
    long delta_time = current_time - previous_time;
    calculateVelocityInstantaneous(delta_time, diff_count_a, diff_count_b);
    if (new_motor_count_a != last_motor_count_a || new_motor_count_b != last_motor_count_b ) {
        last_motor_count_a = new_motor_count_a;
        last_motor_count_b = new_motor_count_b;
    }
    calculateDistanceTotal();
    previous_time = current_time;
}

double Odometry::getDistanceTraveled() {
    return distance;
}

double Odometry::getVelocity() {
    return current_velocity;
}

void Odometry::calculateDistanceTotal() {
     // Converting raw counts into revolutions
    double total_revolutions_a = last_motor_count_a / (double) COUNTS_PER_REVOLUTION;
    double total_revolutions_b = last_motor_count_b / (double) COUNTS_PER_REVOLUTION;
    double distance_a = total_revolutions_a * WHEEL_CIRCUMFERENCE;
    double distance_b = total_revolutions_b * WHEEL_CIRCUMFERENCE;
    distance = (distance_a + distance_b) / 2;
}

void Odometry::calculateVelocityInstantaneous(long delta_time, long delta_a, long delta_b) {
    angular_velocity_a = delta_a/delta_time;
    angular_velocity_b = delta_b/delta_time;
    double avg_angular_velocity = (angular_velocity_a + angular_velocity_b)/2;
    current_velocity = avg_angular_velocity * WHEEL_RADIUS;
}

void Odometry::calculateHeadingInstantaneous(long delta_a, long delta_b) {
    // Converting raw counts into revolutions
    double revolutions_a = delta_a/COUNTS_PER_REVOLUTION;
    double revolutions_b = delta_b/COUNTS_PER_REVOLUTION;
    double delta_distance_a = revolutions_a * WHEEL_CIRCUMFERENCE;
    double delta_distance_b = revolutions_b * WHEEL_CIRCUMFERENCE;
    theta = (delta_distance_a + delta_distance_b)/WHEEL_TRACK;
}
