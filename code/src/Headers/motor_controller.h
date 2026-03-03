#ifndef MOTOR_CONTROLLER_HPP
#define MOTOR_CONTROLLER_HPP

class MotorController {
public:
    MotorController();

    bool init();
    bool commandMotor(int motor_id, double thrust);
};

#endif