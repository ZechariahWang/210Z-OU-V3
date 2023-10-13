#include "iostream"
#include "api.h"

class LinearMotionProfiler{
    private:
        bool init;
    public:
        double time_to_target_vel;
        double distance_to_target_vel;


        double global_distance;
        double global_acceleration;
        double global_max_velocity;
        double global_init_pos;

        void trapezoidal_reset_values();
        void trapezoidal_calculate_initial_kinematic_values(pros::Motor& reference_left_motor, pros::Motor& reference_right_motor, double targetPosition, double maxVelocity, double acceleration);
        void trapezoidal_calculate_motion_profile(pros::Motor& reference_left_motor, pros::Motor& reference_right_motor);

        void s_curve_reset_values();
        void s_curve_calculate_initial_kinematic_values(pros::Motor& reference_left_motor, pros::Motor& reference_right_motor, double targetPosition, double maxVelocity, double acceleration);
        void s_curve_calculate_motion_profile(pros::Motor& reference_left_motor, pros::Motor& reference_right_motor);

};