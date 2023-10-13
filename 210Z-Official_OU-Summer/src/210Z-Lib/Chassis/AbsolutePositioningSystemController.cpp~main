/**
 * @file Odometry.cpp
 * @author Zechariah Wang
 * @brief Odometry logic for global position tracking within robot using cartesian coordinates
 * @version 0.1
 * @date 2023-07-4
 * 
 */

#include "main.h"
#include "vector"
#include "variant"
#include "array"

double    global_robot_x; // global X
double    global_robot_y; // global Y
double    globalTheta;    // global theta

/**
 * @brief The current theta of the robot wrapped to 360 degrees
 * @return angle wrapped to 360 degrees from raw IMU sensor data
 */

int current_robot_heading() {
  globalTheta = fmod(imu_sensor.get_rotation(), 360);
  while (globalTheta < 0) {
    globalTheta += 360;
  }
  while (globalTheta > 360) {
    globalTheta -= 360;
  }
  return globalTheta; 
}

/**
 * @brief Set the value of drivetrain specifics
 * 
 */

void Odometry::set_horizontal_tracker_specs(double diameter, double offset){
  odom.global_horizontal_diameter = diameter;
  odom.global_horizontal_offset = offset;
}
void Odometry::set_vertical_tracker_specs(double diameter, double offset){
  odom.global_vertical_diameter = diameter;
  odom.global_vertical_offset = offset;
}

double Odometry::get_horizontal_offset(){ return odom.global_horizontal_offset; }
double Odometry::get_vertical_offset() { return odom.global_vertical_offset; }

int auxValue(){ return horizontal_rotation_sensor.get_position() * 3 / 500; }
int forwardVal(){ return -vertical_auxiliary_sensor.get_value(); }

double Odometry::get_distance_travelled(bool vertical, bool horizontal){
  if (vertical == true && horizontal == false){ // Is a vertical sensor aka an encoder wheel
    return (double(-vertical_auxiliary_sensor.get_value()) * odom.global_vertical_diameter * M_PI / 360);
  }
  else if (vertical == false && horizontal == true){ // Is a horizontal sensor aka a rotation sensor
    return (double(horizontal_rotation_sensor.get_position()) * odom.global_horizontal_diameter * M_PI / 36000);
  }
  else{ // how is the wheel both an encoder and rotation sensor bruh
    std::cout << "ERROR: Wheel type cannot be both an encoder and and rotation sensor " << std::endl;
    return 0;
  }
}

float getEncoderDistanceTraveled() { return (float(vertical_auxiliary_sensor.get_value()) * 2.75 * M_PI / 360); }
float getRotationDistanceTraveled() { return (float(horizontal_rotation_sensor.get_position()) * 2.75 * M_PI / 36000); }

/**
 * @brief Main Odometry logic for global robot position tracking
 * 
 */

float odom_heading;
float previousVertical1 = 0;
float prevVertical = 0;
float prevHorizontal = 0;
float previousHorizontal1 = 0;
float prevImu = 0;
void Odometry::odometry_position_update() {
    float vertical1Raw = 0;
    float horizontal1Raw = 0;
    float imuRaw = 0;
    vertical1Raw = getEncoderDistanceTraveled();
    horizontal1Raw = getRotationDistanceTraveled();
    imuRaw = imu_sensor.get_rotation() * M_PI / 180;

    float deltaVertical1 = vertical1Raw - previousVertical1;
    float deltaHorizontal1 = horizontal1Raw - previousHorizontal1;
    float deltaImu = imuRaw - prevImu;

    previousVertical1 = vertical1Raw;
    previousHorizontal1 = horizontal1Raw;
    prevImu = imuRaw;

    float heading = odom_heading;
    heading += deltaImu;
    float deltaHeading = heading - odom_heading;
    float avgHeading = odom_heading + deltaHeading / 2;

    float rawVertical = 0;
    float rawHorizontal = 0;
    rawVertical = getEncoderDistanceTraveled();
    rawHorizontal = getRotationDistanceTraveled();
    float horizontalOffset = 0;
    float verticalOffset = 0;
    verticalOffset = 3.9;
    horizontalOffset = -1.18;

    float deltaX = 0;
    float deltaY = 0;
    deltaY = rawVertical - prevVertical;
    deltaX = rawHorizontal - prevHorizontal;
    prevVertical = rawVertical;
    prevHorizontal = rawHorizontal;

    float localX = 0;
    float localY = 0;
    if (deltaHeading == 0) { 
        localX = deltaX;
        localY = deltaY;
    } else {
        localX = 2 * sin(deltaHeading / 2) * (deltaX / deltaHeading + horizontalOffset);
        localY = 2 * sin(deltaHeading / 2) * (deltaY / deltaHeading + verticalOffset);
    }

    global_robot_x += localY * sin(avgHeading);
    global_robot_y += localY * cos(avgHeading);
    global_robot_x += localX * -cos(avgHeading);
    global_robot_y += localX * sin(avgHeading);
    odom_heading = heading;
}



