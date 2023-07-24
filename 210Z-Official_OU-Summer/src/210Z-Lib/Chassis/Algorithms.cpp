/**
 * @file Algorithms.cpp
 * @author Zechariah Wang
 * @brief Movement Algorithms involving global cartesian coordinates. Includes MTP, MTRP, Boomerang, and TTP/STP
 * @version 0.1
 * @date 2023-07-04
 * 
 */

#include "main.h"
#include "vector"
#include "array"
#include "iostream"
#include "algorithm"

/**
 * @brief Motion Algorithm Class Helper Functions move to point class material theme ocean
 * 
 */

void FeedbackControl::set_constants(const double t_kp, const double r_kp, const double f_tt, const double t){ // Set constants
  mtp.t_kp = t_kp;
  mtp.r_kp = r_kp;
  mtp.target_final_tol = f_tt;
  mtp.target_tol = t;
}

void FeedbackControl::reset_mtp_constants(){ // Reset values
  mtp.distance = 0;
  mtp.alpha = 0;
  mtp.t_error = 0;
  mtp.beta = 0;
  mtp.iterator = 0;
}

FeedbackControl::FeedbackControl(){
  mtp.t_local_kp = 13;
  mtp.t_local_derivative          = 0;
  mtp.t_local_integral            = 0;
  mtp.t_local_tolerance           = 3;
  mtp.t_local_error               = 0;
  mtp.t_local_previouserror       = 0;
  mtp.t_local_multiplier          = 3000;
  mtp.t_local_averageposition     = 0;
  mtp.t_local_averageHeading      = 0;
  mtp.t_local_FailSafeCounter     = 0;
  mtp.t_local_threshholdcounter   = 0;
}

/**
 * @brief Returns a value for a simple PID controller to be used within the MTP algorithm
 * 
 * @param t_theta target theta
 * @return PID value for a simple turn
 */

double local_rotation_pid(double t_theta){
  utility::fullreset(0, false);
  mtp.t_local_error = 0;
  mtp.t_local_previouserror = 0;
  mtp.t_local_integral = 0;
  mtp.t_local_derivative = 0;
  mtp.t_local_FailSafeCounter = 0;
  mtp.t_local_averageHeading = imu_sensor.get_rotation(); 
  mtp.t_local_error = t_theta - mtp.t_local_averageHeading; 
  mtp.t_local_integral += mtp.t_local_error; 
  if (mtp.t_local_error == 0 || mtp.t_local_error > t_theta) { mtp.t_local_integral = 0; }
  mtp.t_local_derivative = mtp.t_local_error - mtp.t_local_previouserror; 
  mtp.t_local_previouserror = mtp.t_local_error;
  double voltage = (mtp.t_local_error * mtp.t_kp * 0.01) * 94; 
  if(fabs(mtp.t_local_error) < mtp.t_local_tolerance){ mtp.t_local_threshholdcounter++; }
  else{ mtp.t_local_threshholdcounter = 0; }
  if (fabs(mtp.t_local_error - mtp.t_local_previouserror) < 0.3) { mtp.t_local_FailSafeCounter++; }
  else { mtp.t_local_FailSafeCounter = 0; }
  return voltage;
}

/**
 * @brief Find min angle to reach a target angle when wrapped to 360 degrees
 * 
 * @param targetHeading the target angle the robot reaches to turn to
 * @param currentrobotHeading the current robot angle
 * @return shortest path needed to reach desired angle
 */

double find_min_angle(const int16_t targetHeading, const int16_t currentrobotHeading){
  double turnAngle = targetHeading - currentrobotHeading;
  if (turnAngle > 180 || turnAngle < -180){ turnAngle = turnAngle - (utility::sgn(turnAngle) * 360); }
  return turnAngle;
}

/**
 * @brief Find the min distance between two angles
 * 
 * @param angle1 Initial angle 
 * @param angle2 Secondary Angle
 * @param radians Are the angles in radians?
 */

double get_min_angle_error(float angle1, float angle2, bool radians){
    float max = radians ? 2 * M_PI : 360;
    float half = radians ? M_PI : 180;
    angle1 = fmod(angle1, max);
    angle2 = fmod(angle2, max);
    float error = angle1 - angle2;
    if (error > half) error -= max;
    else if (error < -half) error += max;
    return error;
}

double get_angular_error(double target_x, double target_y){
  double x = target_x - global_robot_x;
  double y = target_y - global_robot_y;
  double delta_theta = atan2(x, y) * 180 / M_PI - current_robot_heading();
  while (fabs(delta_theta) > 180){
    delta_theta -= 360 * delta_theta / fabs(delta_theta);
  }
  return delta_theta;
}

double get_distance_error(double d_target_x, double d_target_y){
	double x = d_target_x;
	double y = d_target_y;
	y -= global_robot_y;
	x -= global_robot_x;
	return sqrt(x * x + y * y);
}

/**
 * @brief Convert angle to desired unit
 * 
 * @param angle the current angle
 * @return angle in desired unit (degree or radians)
 */

int16_t radian_to_degrees_converter(const double angle) { return angle * 180 / M_PI; } // convert radian to degrees
int16_t degrees_to_radians_converter(const double angle){ return angle * M_PI / 180; } // Convert degrees to radian

/**
 * @brief MTP Algorithm. Move to a desired coordinate position while facing a desired angle. Can only be used with mecanum drives
 * 
 * @param targetX the target x coordinate
 * @param targetY the target y coordinate
 * @param targetTheta the target angle in degrees
 * @param translationSpeed max movement speed
 * @param rotationSpeed max rotation speed
 */ 

void FeedbackControl::simultaneous_mov_executor(double targetX,
                                                    double targetY,
                                                    double targetTheta,
                                                    double translationSpeed,
                                                    double rotationSpeed){

  double previousDriveError = 0; double previousTurnError = 0;
  while (true){
    double theta = current_robot_heading() * M_PI / 180;
    double driveError = sqrt(pow(targetX - global_robot_x, 2) + pow(targetY - global_robot_y, 2));
    double positionHypo = sqrt(pow(global_robot_x, 2) + pow(global_robot_y, 2));
    double driveOutput = (driveError * 8) + ((driveError - previousDriveError) * 1.3);

    double turnError = (-theta - targetTheta);
    turnError = atan2f(sinf(turnError), cosf(turnError));
    double turnOutput = local_rotation_pid(targetTheta);

    double angleDesired = atan2f(targetX - global_robot_x, targetY - global_robot_y);
    double angleDrive = (angleDesired - theta);
    angleDrive = atan2f(sinf(angleDrive), cosf(angleDrive));

    double velDrive = driveOutput * cos(angleDrive); 
    double velStrafe = driveOutput * sin(angleDrive);

    double speedFL; double speedBL;
    double speedFR; double speedBR;

    if(fabs(driveError) < 5 && fabs(turnError) < 3){ utility::stop(); break; }
    else{
      speedFL = velDrive + velStrafe + turnOutput; speedBL = velDrive - velStrafe + turnOutput;
      speedFR = velDrive - velStrafe - turnOutput; speedBR = velDrive + velStrafe - turnOutput;
    }

    dt_front_left.move_velocity((speedFL)); dt_rear_left.move_velocity((speedBL));
    dt_front_right.move_velocity((speedFR)); dt_rear_right.move_velocity((speedBR));

    previousTurnError = turnError; 
    previousDriveError = driveError;
    pros::delay(10);
  }
}

/**
 * @brief move to a specific position at a specific angle
 * 
 * @param targetX the target x coordinate
 * @param targetY the target y coordinate
 * @param targetHeading the target angle in degrees
 * @param radius the radius of the arc
 */

void FeedbackControl::move_to_reference_pose(const double targetX,
                                                 const double targetY,
                                                 const double targetHeading,
                                                 const double radius){

  FeedbackControl Auton_Framework;
  mtp.reset_mtp_constants();
  while (true){
    double abstargetAngle = atan2f(targetX - global_robot_x, targetY - global_robot_y) * 180 / M_PI;
    if (abstargetAngle < 0){ abstargetAngle += 360; }

    mtp.distance = sqrt(pow(targetX - global_robot_x, 2) + pow(targetY - global_robot_y, 2));
    mtp.alpha = find_min_angle(abstargetAngle, targetHeading);
    mtp.t_error = find_min_angle(abstargetAngle, current_robot_heading());
    mtp.beta = atan(radius / mtp.distance) * 180 / M_PI;

    if (alpha < 0){ beta = -beta;}
    if (fabs(alpha) < fabs(beta)){ mtp.r_error = mtp.t_error + alpha; }
    else{ mtp.r_error = mtp.t_error + beta; }

    if (mtp.r_error > 180 || mtp.r_error < -180){ mtp.r_error = mtp.r_error - (utility::sgn(mtp.r_error) * 360); }

    double linearVel = mtp.t_kp * mtp.distance;
    double turnVel = mtp.r_kp * mtp.r_error;
    double closetoTarget = false;

    if (mtp.distance < mtp.target_tol){ closetoTarget = true;}
    if (closetoTarget){
      linearVel = mtp.t_kp * mtp.distance * utility::sgn(cos(mtp.r_error * M_PI / 180));
      mtp.r_error = find_min_angle(targetHeading, current_robot_heading());
      turnVel = mtp.r_kp * atan(tan(mtp.r_error * M_PI / 180)) * 180 / M_PI;
    }

    int16_t left_volage = linearVel + turnVel;
    int16_t right_voltage = linearVel - turnVel;
    int16_t linError_f = sqrt(pow(targetX - global_robot_x, 2) + pow(targetY - global_robot_y, 2));

    utility::leftvoltagereq(left_volage * (12000.0) / 127);
    utility::rightvoltagereq(right_voltage * (12000.0 / 127));

    if (fabs(sqrt(pow(targetX - global_robot_x, 2) + pow(targetY - global_robot_y, 2))) < mtp.target_final_tol)
    { 
      utility::stop();
      break;
    }
    else {mtp.iterator = 0;}
    if (mtp.iterator > 10) {
      utility::stop();
      break;
    }
    pros::delay(10);
  }
}

/**
 * @brief Swing to a specific coordinate position. Used for Pure Pursuit, and other motion algorithms
 * 
 * @param targetX the target x coordinate
 * @param targetY the target y coordinate
 * @param swingDamper the amount the arc is dampered by
 */

void FeedbackControl::swing_to_point(const double tx,
                                         const double ty,
                                         const double swingDamper){
                                          
  double defaultVoltage = 40;
  double abstargetAngle = atan2f(tx - global_robot_x, ty - global_robot_y) * 180 / M_PI;
  if (abstargetAngle < 0){ abstargetAngle += 360; }
  double targetTheta = find_min_angle(abstargetAngle, imu_sensor.get_rotation()) * 100;
  utility::leftvoltagereq((defaultVoltage * (12000.0 / 127)) + targetTheta);
  utility::rightvoltagereq((defaultVoltage * (12000.0 / 127)) - targetTheta);
}

// Curve to desired point
void curve_to_point(const double tx, const double ty, const double curveDamper){
  utility::leftvoltagereq(100 * (12000.0 / 127));
  utility::rightvoltagereq(100 * (12000.0 / 127));
}

// Turn to target coordinate position
void FeedbackControl::TurnToPoint(const int targetX, const int targetY){
  double counter = 0;
  while (true){
    double deltaX = targetX - global_robot_x;
    double deltaY = targetY - global_robot_y;

    double targetTheta = fmod((atan2f(deltaY, deltaX) * 180 / M_PI), 360);
    double target_heading = get_min_angle_error(targetTheta, current_robot_heading(), false);

    utility::leftvoltagereq(target_heading * (12000.0 / 127) * 5);
    utility::rightvoltagereq(-target_heading * (12000.0 / 127) * 5);
    std::cout << "in ttp loop" << std::endl;

    if (fabs(target_heading) < 3){
      counter++;
    }
    if (counter >= 10){
      utility::stop();
      counter = 0;
      return;
    }
    pros::delay(10);
  }
}

void boomerang(double target_x, double target_y, double target_theta, double max_linear_speed, double max_rotation_speed, double d_lead, double kp_linear, double kp_angular) {
    while (true) {
        odom.odometry_position_update();
        double h = std::sqrt(pow(target_x - global_robot_x, 2) + pow(target_y - global_robot_y, 2));
        std::cout << "h: " << h << std::endl;     
        double at = target_theta * M_PI / 180;
        double carrot_point_x = target_x - h * std::sin(at) * d_lead;
        double carrot_point_y = target_y - h * std::cos(at) * d_lead;

        double global_turn_error = get_min_angle_error(target_theta, current_robot_heading(), false);
        double global_linear_error = get_distance_error(target_x, target_y);
        double linear_error = global_linear_error;
        double angular_error = get_angular_error(carrot_point_x, carrot_point_y);

        double linear_speed = linear_error * kp_linear;
        double angular_speed = angular_error * kp_angular;

        if (std::fabs(global_linear_error) < 3) {
      	    rot_r.set_r_constants(6, 0, 45);
	          rot_r.set_rotation_pid(target_theta, 90);
            utility::stop();
            break;
        }

        utility::leftvoltagereq((linear_speed + angular_speed) * (12000.0 / 127));
        utility::rightvoltagereq((linear_speed - angular_speed) * (12000.0 / 127));

        pros::delay(10);
    }
}


void mimic_move_to_point(double target_x,
                   double target_y,
                   double max_linear_speed,
                   double max_rotation_speed, 
                   double kp_linear, 
                   double kp_angular){

    odom.odometry_position_update();
    double angular_error = get_angular_error(target_x, target_y);
    double linear_error = get_distance_error(target_x, target_y);

    double linear_speed = linear_error * kp_linear;
    double angular_speed = angular_error * kp_angular;

    if (linear_speed > max_linear_speed) {
      linear_speed = max_linear_speed;
    }
    if (linear_speed < -max_linear_speed){
      linear_speed = -max_linear_speed;
    }
    if (angular_speed > max_rotation_speed){
      angular_speed = max_rotation_speed;
    }
    if (angular_speed < -max_rotation_speed){
      angular_speed = -max_rotation_speed;
    }

    utility::leftvoltagereq((linear_speed + angular_speed) * (12000.0 / 127));
    utility::rightvoltagereq((linear_speed - angular_speed) * (12000.0 / 127));
}

void move_to_point(double target_x,
                   double target_y,
                   double max_linear_speed,
                   double max_rotation_speed, 
                   double kp_linear, 
                   double kp_angular){

  while (true){
    odom.odometry_position_update();
    double angular_error = get_angular_error(target_x, target_y);
    double linear_error = get_distance_error(target_x, target_y);

    double linear_speed = linear_error * kp_linear;
    double angular_speed = angular_error * kp_angular;

    if (linear_speed > max_linear_speed) {
      linear_speed = max_linear_speed;
    }
    if (linear_speed < -max_linear_speed){
      linear_speed = -max_linear_speed;
    }
    if (angular_speed > max_rotation_speed){
      angular_speed = max_rotation_speed;
    }
    if (angular_speed < -max_rotation_speed){
      angular_speed = -max_rotation_speed;
    }

    utility::leftvoltagereq((linear_speed + angular_speed) * (12000.0 / 127));
    utility::rightvoltagereq((linear_speed - angular_speed) * (12000.0 / 127));

    if (fabs(linear_error) < 7){
      utility::stop();
      break;
    }
    pros::delay(10);
  }
}





