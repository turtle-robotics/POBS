#ifndef STATE_ESTIMATOR_HPP
#define STATE_ESTIMATOR_HPP

#include "common_types.hpp"
#include "state_node.hpp"
#include "motor_controller.hpp"


//Stores all logic relating to ensuting to goal state is aquired
class StateEstimator {
public:
    //Starts up the state estimator and declares all the variables
    StateEstimator();

    //Helper function to allow manual reading from a json
    void loadParams();

    //Uses the current gps to reset the kalaman tate estimante
    void resetPosition();


private:
    State* cur_state;
    //This is read only in this function, writen from the node
    State* goal_state;
    KalmanFilter* kalman;
    StateNode* node;
    MotorController* motorcontroller;
};

#endif