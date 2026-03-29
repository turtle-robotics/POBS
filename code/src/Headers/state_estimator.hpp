#ifndef STATE_ESTIMATOR_HPP
#define STATE_ESTIMATOR_HPP

#include "common_types.hpp"
#include "state_node.hpp"
#include "motor_controller.hpp"
#include <string>


//Stores all logic relating to ensuting to goal state is aquired
class StateEstimator {
public:
    //Starts up the state estimator and declares all the variables
    StateEstimator();

    //Helper function to allow manual reading from a json
    void loadParams(std::string filename);

    //Uses the current gps to reset the kalaman tate estimante
    void resetPosition();

    //Uses the file to preform tests
    void testFromFile(std::string filename);

private:
    State* cur_state;
    //This is read only in this function, writen from the node
    State* goal_state;
    KalmanFilter* kalman;
    StateNode* node;
    MotorController* motorcontroller;
};

#endif