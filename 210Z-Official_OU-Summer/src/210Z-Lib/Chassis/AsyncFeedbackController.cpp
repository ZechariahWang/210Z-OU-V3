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
 * @brief Turn to a specific coordinate position
 * 
 * @param targetX the target x coordinate
 * @param targetY the target y coordinate
 */

void FeedbackControl::TurnToPoint(const int targetX, const int targetY){
  double counter = 0;
  while (true){
    double angular_error = utility::get_angular_error(targetX, targetY);
    double angular_speed = angular_error * 5;

    utility::leftvoltagereq(angular_speed * (12000.0 / 127) * 5);
    utility::rightvoltagereq(-angular_speed * (12000.0 / 127) * 5);
    std::cout << "in ttp loop" << std::endl;

    if (fabs(angular_error) < 3){
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

/**
 * @brief Mimic the movement of the standard MTP algorithm. Used for Pure Pursuit
 * 
 * @param target_x the target x coordinate
 * @param target_y the target y coordinate
 * @param max_linear_speed max speed robot can move while doing lateral movements
 * @param max_rotation_speed max speed robot can move while doing rotational movements
 * @param kp_linear linear proportional value
 * @param kp_angular angular proportional value
 */

void mimic_move_to_point(double target_x,
                   double target_y,
                   double max_linear_speed,
                   double max_rotation_speed, 
                   double kp_linear, 
                   double kp_angular){

    odom.odometry_position_update();
    double angular_error = utility::get_angular_error(target_x, target_y);
    double linear_error = utility::get_distance_error(target_x, target_y);

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

/**
 * @brief Move to desired gloal robot position
 * 
 * @param target_x the target x coordinate
 * @param target_y the target y coordinate
 * @param max_linear_speed max speed robot can move while doing lateral movements
 * @param max_rotation_speed max speed robot can move while doing rotational movements
 * @param kp_linear linear proportional value
 * @param kp_angular angular proportional value
 */

void move_to_point(double target_x,
                   double target_y,
                   double max_linear_speed,
                   double max_rotation_speed, 
                   double kp_linear, 
                   double kp_angular){

  while (true){
    odom.odometry_position_update();
    double angular_error = utility::get_angular_error(target_x, target_y);
    double linear_error = utility::get_distance_error(target_x, target_y);

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

void boomerang(double target_x, double target_y, double target_theta, double max_linear_speed, double max_rotation_speed, double d_lead, double kp_linear, double kp_angular) {
    while (true) {
        odom.odometry_position_update();
        double h = std::sqrt(pow(target_x - global_robot_x, 2) + pow(target_y - global_robot_y, 2));
        std::cout << "h: " << h << std::endl;     
        double at = target_theta * M_PI / 180;
        double carrot_point_x = target_x - h * std::sin(at) * d_lead;
        double carrot_point_y = target_y - h * std::cos(at) * d_lead;

        double global_turn_error = utility::get_min_angle_error(target_theta, current_robot_heading(), false);
        double global_linear_error = utility::get_distance_error(target_x, target_y);
        double linear_error = global_linear_error;
        double angular_error = utility::get_angular_error(carrot_point_x, carrot_point_y);

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

double lin_kp = 3;
double ang_kp = 1.5;
void new_boomerang(double pointTarget_x, double pointTarget_y, double angularTarget, double leadPct){
	// previous sensor values
	static double pe_lin = 0;
	static double pe_ang = 0;

	// an angular target > 360 indicates no desired final pose angle
	bool noPose = (angularTarget > 360);
	double carrotPoint_x; 
	double carrotPoint_y; 

	if (noPose) {
		// point movement
		carrotPoint_x = pointTarget_x;
		carrotPoint_y = pointTarget_y;
	} else {
		// pose movement
		double h = utility::get_distance_error(pointTarget_x, pointTarget_y);
		double at = angularTarget * M_PI / 180.0;
		carrotPoint_x = pointTarget_x - h * cos(at) * leadPct,
		carrotPoint_y = pointTarget_y - h * sin(at) * leadPct;
	}

	// get current error
	double lin_error = utility::get_distance_error(pointTarget_x, pointTarget_y);
	double ang_error = utility::get_angular_error(carrotPoint_x, carrotPoint_y);

	// calculate linear speed
	double lin_speed;
	lin_speed = lin_error * lin_kp;

  double ang_speed;
  if (lin_error < 10) {
		if (noPose) {
			ang_speed = 0; // disable turning when close to the point to prevent spinning
		} else {
			// turn to face the finale pose angle if executing a pose movement
			double poseError = (angularTarget * M_PI / 180) - odom_heading;
			while (fabs(poseError) > M_PI)
				poseError -= 2 * M_PI * poseError / fabs(poseError);
			ang_speed = poseError * ang_kp;
		}

		// reduce the linear speed if the bot is tangent to the target
		lin_speed *= cos(ang_error);

	} else {
		ang_speed = ang_error * ang_kp;
	}

	// add speeds together
	double left_speed = lin_speed - ang_speed;
	double right_speed = lin_speed + ang_speed;

	utility::leftvoltagereq(left_speed);
	utility::rightvoltagereq(right_speed);
}






