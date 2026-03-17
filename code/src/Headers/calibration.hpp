#ifndef CALIBRATION_NODE_HPP
#define CALIBRATION_NODE_HPP

#include "common_types.hpp"

//The goal of the calibration node is to send needed movements then wait on the ros network and listen for new movements, then check that thse movements are logical for the setup.
class CalibrationNode {
public:
    //Initalizes all of the ros communication
    CalibrationNode();

    //This sends the actions on the ros network
    bool sendCalibrationActions();

    //This what runs when a state is recived stores the state
    void stateCallback(const State& incoming_state);

    //This is the code that checks to see if the past actions are logical
    bool verifyMotorPositions();

    //This function checks that all sensors are providing data
    bool verifySensorData();

private:
    //Stores the last 40 states for use
    State recent_states[40];
};


#endif