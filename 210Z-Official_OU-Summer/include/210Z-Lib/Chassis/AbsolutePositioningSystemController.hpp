#include "iostream"

extern double global_robot_x;
extern double global_robot_y;

int current_robot_heading();

class Odometry{
    private:
        bool init = true;
    public:

        float global_horizontal_diameter;
        float global_vertical_diameter;
        float global_horizontal_offset;
        float global_vertical_offset;

        void set_horizontal_tracker_specs(double diameter, double offset);
        void set_vertical_tracker_specs(double diameter, double offset);

        double get_distance_travelled(bool vertical, bool horizontal);

        double get_horizontal_offset();
        double get_vertical_offset();
        void odometry_position_update();
};

extern float odom_heading;

extern double heading_very_new;

void odomTask();
extern float odom_heading;
void update_odom_new();
void twoSensorOdom();


void odom_task_new();
void odomTask_LOL();
