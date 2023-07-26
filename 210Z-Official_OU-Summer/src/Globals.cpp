/**
 * @file Globals.cpp
 * @author Zechariah Wang
 * @brief Robot physical parts declaration
 * @version 0.1
 * @date 2023-07-04
 */

#include "pros/adi.hpp"
#include "main.h"

bool GPS_ENABLED;
pros::ADIEncoder vertical_auxiliary_sensor('g', 'h', false);
pros::Rotation horizontal_rotation_sensor(8);
pros::Gps gps_sensor(99);
pros::c::gps_status_s_t gpsData;
pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Imu imu_sensor(20);
pros::Motor dt_front_left(17, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor dt_front_right(16, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor dt_rear_left(7, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor dt_rear_right(5, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor dt_middle_left(12, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor dt_middle_right(9, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);

pros::MotorGroup left_side_motors({dt_front_left, dt_middle_left, dt_rear_left});
pros::MotorGroup right_side_motors({dt_front_right, dt_middle_right, dt_rear_right});

TranslationPID mov_t;
RotationPID rot_r;
CurvePID cur_c;
ArcPID arc_a; 
FeedbackControl mtp;
Odometry odom;
LinearMotionProfiler profiler;

match_mov op_mov;

Metrics data_displayer;
Selector data; 

KalmanFilter kal;
Slew slew;
Math math;

pros::ADIAnalogIn cata_sensor('a');
pros::Motor cata_motor(3, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor intake_motor(2, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);
