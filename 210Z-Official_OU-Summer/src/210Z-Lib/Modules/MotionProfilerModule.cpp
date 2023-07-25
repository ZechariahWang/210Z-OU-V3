#include "main.h"

void LinearMotionProfiler::trapezoidal_reset_values(){
    profiler.time_to_target_vel = 0;
    profiler.distance_to_target_vel = 0;

    profiler.global_distance = 0;
    profiler.global_acceleration = 0;
    profiler.global_max_velocity = 0;
    profiler.global_init_pos = 0;
}

void LinearMotionProfiler::trapezoidal_calculate_initial_kinematic_values(pros::Motor& reference_left_motor, pros::Motor& reference_right_motor, double targetPosition, double maxVelocity, double acceleration) {
    reference_left_motor.set_zero_position(0); reference_right_motor.set_zero_position(0);
    double initialPosition = (reference_left_motor.get_position() + reference_right_motor.get_position()) / 2;
    double distance = targetPosition - initialPosition;
    double targetVelocity = utility::sgn(distance) * maxVelocity;
    
    // Calculate the time to reach the target velocity
    double timeToTargetVelocity = targetVelocity / acceleration;

    // Calculate the distance to accelerate to the target velocity
    double distanceToTargetVelocity = 0.5 * acceleration * timeToTargetVelocity * timeToTargetVelocity;

    profiler.time_to_target_vel = timeToTargetVelocity;
    profiler.distance_to_target_vel = distanceToTargetVelocity;

    profiler.global_distance = distance;
    profiler.global_acceleration = acceleration;
    profiler.global_max_velocity = maxVelocity;
    profiler.global_init_pos = initialPosition;
}

void LinearMotionProfiler::trapezoidal_calculate_motion_profile(pros::Motor& reference_left_motor, pros::Motor& reference_right_motor){
    if (fabs(profiler.global_distance) > 2 * profiler.distance_to_target_vel) {
        // Trapezoidal profile
        double timeToDeacceleration = fabs(profiler.global_distance) / profiler.global_max_velocity;
        double totalTime = profiler.time_to_target_vel + timeToDeacceleration;

        double accelerationTimePercentage = profiler.time_to_target_vel / totalTime;
        double decelerationTimePercentage = timeToDeacceleration / totalTime;

        // Perform the motion profile
        double startTime = pros::millis();
        double currentTime;
        double currentPosition;

        while (true) {
            currentTime = (pros::millis() - startTime) / 1000;
            if (currentTime >= totalTime) {
                reference_left_motor.move_velocity(0); 
                reference_right_motor.move_velocity(0);
                break;
            } else if (currentTime <= profiler.time_to_target_vel) {
                // Acceleration phase
                currentPosition = profiler.global_init_pos + utility::sgn(profiler.global_distance) * 0.5 * profiler.global_acceleration * currentTime * currentTime;
                reference_left_motor.move_velocity(utility::sgn(profiler.global_distance) * profiler.global_acceleration * currentTime);
                reference_right_motor.move_velocity(utility::sgn(profiler.global_distance) * profiler.global_acceleration * currentTime);
            } else if (currentTime <= totalTime - timeToDeacceleration) {
                // Constant velocity phase
                currentPosition = profiler.global_init_pos + utility::sgn(profiler.global_distance) * (profiler.distance_to_target_vel + profiler.global_max_velocity * (currentTime - profiler.time_to_target_vel));
                reference_left_motor.move_velocity(utility::sgn(profiler.global_distance) * profiler.global_max_velocity);
                reference_right_motor.move_velocity(utility::sgn(profiler.global_distance) * profiler.global_max_velocity);
            } else {
                // Deceleration phase
                double decelerationTime = currentTime - (totalTime - timeToDeacceleration);
                currentPosition = profiler.global_init_pos + utility::sgn(profiler.global_distance) * (profiler.global_distance - 0.5 * profiler.global_acceleration * decelerationTime * decelerationTime);
                reference_left_motor.move_velocity(utility::sgn(profiler.global_distance) * (profiler.global_max_velocity - profiler.global_acceleration * decelerationTime));
                reference_right_motor.move_velocity(utility::sgn(profiler.global_distance) * (profiler.global_max_velocity - profiler.global_acceleration * decelerationTime));
            }
            pros::delay(10); 
        }
    } else {
        // Triangular profile (no constant velocity phase)
        double accelerationTime = sqrt(fabs(profiler.global_distance) / profiler.global_acceleration);
        double totalTime = 2 * accelerationTime;

        // Perform the motion profile
        double startTime = pros::millis();
        double currentTime;
        double currentPosition;

        while (true) {
            currentTime = (pros::millis() - startTime) / 1000;
            if (currentTime >= totalTime) {
                reference_left_motor.move_velocity(0); 
                reference_right_motor.move_velocity(0); 
                break;
            } else if (currentTime <= accelerationTime) {
                // Acceleration phase
                currentPosition = profiler.global_init_pos + utility::sgn(profiler.global_distance) * 0.5 * profiler.global_acceleration * currentTime * currentTime;
                reference_left_motor.move_velocity(utility::sgn(profiler.global_distance) * profiler.global_acceleration * currentTime);
            } else {
                // Deceleration phase
                double decelerationTime = currentTime - accelerationTime;
                currentPosition = profiler.global_init_pos + utility::sgn(profiler.global_distance) * (profiler.global_distance - 0.5 * profiler.global_acceleration * decelerationTime * decelerationTime);
                reference_left_motor.move_velocity(utility::sgn(profiler.global_distance) * (profiler.global_max_velocity - profiler.global_acceleration * decelerationTime));
            }
            pros::delay(10); 
        }
    }
}

void LinearMotionProfiler::s_curve_reset_values(){
    profiler.time_to_target_vel = 0;
    profiler.distance_to_target_vel = 0;

    profiler.global_distance = 0;
    profiler.global_acceleration = 0;
    profiler.global_max_velocity = 0;
    profiler.global_init_pos = 0;
}

void LinearMotionProfiler::s_curve_calculate_initial_kinematic_values(pros::Motor& reference_left_motor, pros::Motor& reference_right_motor, double targetDistance, double maxVelocity, double maxAcceleration) {
    reference_left_motor.set_zero_position(0); reference_right_motor.set_zero_position(0);
    double initialPosition = (reference_left_motor.get_position() + reference_right_motor.get_position()) / 2;
    double distance = targetDistance;

    double timeToTargetVelocity = maxVelocity / maxAcceleration;
    double distanceToTargetVelocity = 0.5 * maxAcceleration * timeToTargetVelocity * timeToTargetVelocity;
    double timeToTargetPosition = sqrt(fabs(distance) / maxAcceleration) + timeToTargetVelocity;

    double startTime = pros::millis();
    double currentTime;
    double currentPosition;
    double currentVelocity;

    profiler.time_to_target_vel = timeToTargetVelocity;
    profiler.distance_to_target_vel = distanceToTargetVelocity;

    profiler.global_distance = distance;
    profiler.global_acceleration = maxAcceleration;
    profiler.global_max_velocity = maxVelocity;
    profiler.global_init_pos = initialPosition;
}

void s_curve_calculate_motion_profile(pros::Motor& reference_left_motor, pros::Motor& reference_right_motor, double targetDistance, double maxVelocity, double maxAcceleration) {
    double initialPosition = (reference_left_motor.get_position() + reference_right_motor.get_position()) / 2;
    double distance = targetDistance;

    double timeToTargetVelocity = maxVelocity / maxAcceleration;
    double distanceToTargetVelocity = 0.5 * maxAcceleration * timeToTargetVelocity * timeToTargetVelocity;
    double timeToTargetPosition = sqrt(fabs(distance) / maxAcceleration) + timeToTargetVelocity;

    double startTime = pros::millis();
    double currentTime;
    double currentPosition;
    double currentVelocity;

    profiler.time_to_target_vel = timeToTargetVelocity;
    profiler.distance_to_target_vel = distanceToTargetVelocity;

    profiler.global_distance = distance;
    profiler.global_acceleration = maxAcceleration;
    profiler.global_max_velocity = maxVelocity;
    profiler.global_init_pos = initialPosition;

    while (true) {
        currentTime = (pros::millis() - startTime) / 1000.0; 
        if (currentTime >= timeToTargetPosition) {
            reference_left_motor.move_velocity(0);
            reference_right_motor.move_velocity(0);
            break;
        } 
        else {
            // Calculate the position, velocity, and acceleration using a cubic polynomial
            double t = currentTime; double t2 = t * t; double t3 = t2 * t;
            currentPosition = initialPosition + utility::sgn(distance) * (0.5 * maxAcceleration * t3);
            currentVelocity = utility::sgn(distance) * (1.5 * maxAcceleration * t2);
            reference_left_motor.move_velocity(currentVelocity);
            reference_right_motor.move_velocity(currentVelocity);

            pros::delay(10); 
        }
    }
}