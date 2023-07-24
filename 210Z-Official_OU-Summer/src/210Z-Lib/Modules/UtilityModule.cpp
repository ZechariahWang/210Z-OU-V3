/**
 * @file UtilityModule.cpp
 * @author Zechariah Wang
 * @brief Helper functions and utility namespace assets
 * @version 0.1
 * @date 2023-07-04
 */

#include "main.h"

namespace utility{
  int sgn(double num){ return (num < 0) ? -1 : ((num > 0) ? 1 : 0); }
  double clamp(double num, double min, double max){
    if (num > max){ return max; }
    if (num < min) { return min; }
    else { return num; }
  }
  void stop(){
    dt_front_left.move_voltage(0); dt_rear_left.move_voltage(0); dt_middle_left.move_voltage(0);
    dt_front_right.move_voltage(0); dt_rear_right.move_voltage(0); dt_middle_right.move_voltage(0);
  }
  void stop_v(){
    dt_front_left.move_velocity(0); dt_middle_left.move_velocity(0); dt_rear_left.move_velocity(0);
    dt_front_right.move_velocity(0); dt_middle_right.move_velocity(0); dt_rear_right.move_velocity(0); 
  }

  void leftvelreq(double velocity){ dt_front_left.move_voltage(velocity); dt_middle_left.move_voltage(velocity); dt_rear_left.move_voltage(velocity); }
  void rightvelreq(double velocity){ dt_front_right.move_voltage(velocity); dt_middle_right.move_voltage(velocity); dt_rear_right.move_voltage(velocity); }
  void leftvoltagereq(double voltage){ dt_front_left.move_voltage(voltage); dt_middle_left.move_voltage(voltage); dt_rear_left.move_voltage(voltage); }
  void rightvoltagereq(double voltage){ dt_front_right.move_voltage(voltage); dt_middle_right.move_voltage(voltage); dt_rear_right.move_voltage(voltage); }

  void fullreset(double resetval, bool imu){
    dt_front_left.set_zero_position(resetval); dt_middle_left.set_zero_position(resetval); dt_rear_left.set_zero_position(resetval);
    dt_front_right.set_zero_position(resetval); dt_middle_right.set_zero_position(resetval); dt_rear_right.set_zero_position(resetval);
    if (imu == true){ imu_sensor.tare_rotation(); }
  }

  double get_x(){ return global_robot_x; } 
  double get_y(){ return global_robot_y; }

  void set_x(double x) { global_robot_x = x; }
  void set_y(double y) { global_robot_y = y; }

}

void FeedbackControl::overRideCoordinatePos(double new_gx, double new_gy){ utility::set_x(new_gx); utility::set_y(new_gy); }

