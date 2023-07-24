#include "iostream"

namespace utility { // util
    int sgn(double num);
    double clamp(double num, double min, double max);
    void stop();
    void stop_v();
    void leftvelreq(int velocity);
    void rightvelreq(int velocity);
    void leftvoltagereq(double voltage);
    void rightvoltagereq(double voltage);
    void fullreset(double resetval, bool imu);
    double get_x();
    double get_y();
    void set_x(double x);
    void set_y(double y);

}
