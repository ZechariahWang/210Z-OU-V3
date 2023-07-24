/**
 * @file SolenoidModule.cpp
 * @author Zechariah Wang
 * @brief Create a new solenoid controller for pistons on the robot
 * @version 0.1
 * @date 2023-07-04
 */

#include "main.h"

Solenoid::Solenoid(std::uint8_t iPort, bool initial_state) : piston(iPort), state(initial_state){
    piston.set_value(state);
}

Solenoid::Solenoid(pros::ext_adi_port_pair_t iPortPair, bool initial_state) : piston(iPortPair), state(initial_state) {
    piston.set_value(state);
}

void Solenoid::toggle(){
    state = !state;
    piston.set_value(state);
}

void Solenoid::set(bool iState){
    state = iState;
    piston.set_value(iState);
}

bool Solenoid::get_state() const{
    return state;
}

