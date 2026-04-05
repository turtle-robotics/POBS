#ifndef MOTOR_CONTROLLER_HPP
#define MOTOR_CONTROLLER_HPP

#include "common_types.hpp"

//This takes in the movements and sends the PID signals
class MotorController {
public:
    //Starts up the controller code
    MotorController(State* goal_state, State* current_state);

    //sets the PWMs of the motors from a direct thrust returns false on failure
    bool commandMotor(int motor_id, double thrust);

    //Sets the PWMs to roll the system
    StateTransition commandRoll(double roll_change);

    //Sets the PWMs to pitch the system
    StateTransition commandPitch(double pitch_change);
    
    //Sets the PWMs to yaw the system
    StateTransition commandYaw(double yaw_change);

    //Sets the PWMs to move the system forward
    StateTransition commandForward(double acceleration);

private:

    //converter
    double umsToDuty(int ums);

    double thrusts[8];
    double pwms[8];
    Expression thrust_to_pwms[8];
    //This is read only in this function modified in the estimator
    State* goal_state;
    //This is read only in this function modified in the estimator
    State* current_state;
};

#endif