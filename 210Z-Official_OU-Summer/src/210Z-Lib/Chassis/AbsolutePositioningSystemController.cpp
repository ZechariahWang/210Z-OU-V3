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
    return (double(-horizontal_rotation_sensor.get_position()) * odom.global_horizontal_diameter * M_PI / 36000);
  }
  else{ // how is the wheel both an encoder and rotation sensor bruh
    std::cout << "ERROR: Wheel type cannot be both an encoder and and rotation sensor " << std::endl;
    return 0;
  }
}

float getEncoderDistanceTraveled() { return (float(vertical_auxiliary_sensor.get_value()) * 2.75 * M_PI / 360); }
float getRotationDistanceTraveled() { return (float(horizontal_rotation_sensor.get_position()) * 2.75 * M_PI / 36000); }

// odom position values
double heading_very_new;
 
// previous values
double prev_left_pos = 0;
double prev_right_pos = 0;
double prev_middle_pos = 0;
double prev_heading = 0;

double tpi = 1;
double middle_tpi = 1;
 
void odom_task_new() {
  double right_pos = -getRotationDistanceTraveled();
  double middle_pos = 0;

  double delta_right = (right_pos - prev_right_pos) / tpi;
  double delta_middle = (middle_pos - prev_middle_pos) / middle_tpi;
 
  // calculate new heading
  double delta_angle;
  heading_very_new = -imu_sensor.get_rotation() * M_PI / 180.0;
  delta_angle = heading_very_new - prev_heading;
 
  // store previous positions
  prev_right_pos = right_pos;
  prev_middle_pos = middle_pos;
  prev_heading = heading_very_new;
 
  // calculate local displacement
  double local_x;
  double local_y;
 
  if (delta_angle) {
    double i = sin(delta_angle / 2.0) * 2.0;
    local_x = (delta_right / delta_angle - 3) * i;
    local_y = (delta_middle / delta_angle + 1.2) * i;
  } else {
    local_x = delta_right;
    local_y = delta_middle;
  }
 
  double p = heading_very_new - delta_angle / 2.0; // global angle
 
  // convert to absolute displacement
  global_robot_x += cos(p) * local_x - sin(p) * local_y;
  global_robot_y += cos(p) * local_y + sin(p) * local_x;

	data_displayer.output_sensor_data(); // Display robot stats and info
	data_displayer.display_data();
	data_displayer.output_misc_data();
}

int auxVal(){ return horizontal_rotation_sensor.get_position() * 3 / 500; }
int forVal(){ return -vertical_auxiliary_sensor.get_value(); }

int counter(0);
double currentLeft(0), lastLeft(0), deltaLeft(0);
double currentRight(0), lastRight(0), deltaRight(0);
double currentCenter(0), lastCenter(0), deltaCenter(0);
double theory(0), deltaTheory(0);
double lastTheta(0), deltaTheta(0);
double rotationTheta(0), totalRotationTheta(0);
double deltaX(0), deltaY(0);

double currentForwards, previosuForwards, deltaFowards, deltaTheory2,theory2;
double currentOTheta, deltaOTheta, lastOTheta, odom_counter;

void twoSensorOdom(){
  char buffer[300];
  double theta = current_robot_heading() * M_PI / 180;
  double x = cos(current_robot_heading() * M_PI / 180 + M_PI);
  double y = sin(current_robot_heading() * M_PI / 180 + M_PI);

  if (fmod(counter, 3) < 1)
  {
    theta = std::abs(atan2f(y, x) + M_PI); // theta is in radians
  }

  currentForwards = getRotationDistanceTraveled();
  currentCenter =  getEncoderDistanceTraveled();
  currentOTheta = imu_sensor.get_rotation();

  deltaFowards = currentForwards - previosuForwards;
  deltaCenter = currentCenter - lastCenter;
  deltaOTheta = currentOTheta - lastOTheta;

  deltaTheory = deltaOTheta*1.09*0.79*1.115*1.413*0.9566*.9436*1.066*.867*2.26*0.0677*0.8366*1.069*1.16*0.5611*.927/1.33/1.307;
  deltaTheory2 = deltaOTheta*1.35*0.103*0.9566*.9436*1.066*.867*2.26*0.0677*0.8366*1.069*1.16*0.5611*.927/1.33/1.307;
  theory += deltaTheory;
  theory2 += deltaTheory2;

  deltaX = ((deltaFowards) * 1.0f * -sinf(-theta));
  deltaY = ((deltaFowards) * 1.0f * cosf(-theta));

  global_robot_x += deltaX;
  global_robot_y += deltaY;

  previosuForwards = currentForwards;
  lastCenter = currentCenter;
  lastOTheta = currentOTheta;
  lastTheta = theta;

	data_displayer.output_sensor_data(); // Display robot stats and info
	data_displayer.display_data();
	data_displayer.output_misc_data();

}






