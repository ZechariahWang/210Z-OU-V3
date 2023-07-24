#include "pros/adi.hpp"

#pragma once
#include "main.h"

extern bool GPS_ENABLED;
extern pros::Controller controller;
extern pros::Motor dt_front_left;
extern pros::Motor dt_front_right;
extern pros::Motor dt_middle_left;
extern pros::Motor dt_middle_right;
extern pros::Motor dt_rear_left;
extern pros::Motor dt_rear_right;

extern pros::ADIEncoder vertical_auxiliary_sensor;
extern pros::Rotation horizontal_rotation_sensor;
extern pros::Imu imu_sensor;
extern pros::Gps gps_sensor;
extern pros::c::gps_status_s_t gps_data;

extern pros::MotorGroup left_side_motors;
extern pros::MotorGroup right_side_motors;

extern KalmanFilter kal;
extern TranslationPID mov_t;
extern match_mov op_mov;
extern Metrics data_displayer;
extern Selector data; 
extern Slew slew;
extern RotationPID rot_r;
extern CurvePID cur_c;
extern ArcPID arc_a; 
extern FeedbackControl mtp;
extern Odometry odom;
extern Math math;


