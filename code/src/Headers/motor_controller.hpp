#ifndef MOTOR_CONTROLLER_HPP
#define MOTOR_CONTROLLER_HPP

#include "parametric_expressions.hpp"

//This takes in the movements and sends the PID signals
class MotorController {
public:
    //Starts up the controller code
    MotorController();

    //sets the PWMs of the motors from a direct thrust returns false on failure
    bool commandMotor(int motor_id, double thrust);

    //Sets the PWMs to roll the system
    bool commandRoll(double roll_change);

    //Sets the PWMs to pitch the system
    bool commandPitch(double pitch_change);
    
    //Sets the PWMs to yaw the system
    bool commandYaw(double yaw_change);

    //Sets the PWMs to move forward the system
    bool commandForward(double acceleration);

private:
    double thrusts[8];
    double pwms[8];
    Expression thrust_to_pwms[8];
};

#endif