#include "iostream"
#include "api.h"

class Solenoid{
    pros::ADIDigitalOut piston;
    bool state;

    public:
        Solenoid(std::uint8_t iPort, bool initial_state = false);
        Solenoid(pros::ext_adi_port_pair_t iPortPair, bool initial_state = false);
        ~Solenoid() = default;
        void toggle();
        void set(bool iState);
        bool get_state() const;
};
