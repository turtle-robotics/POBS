#ifndef CALIBRATION_NODE_HPP
#define CALIBRATION_NODE_HPP

#include "common_types.h"

class CalibrationNode {
public:
    CalibrationNode();

    bool sendCalibrationActions();
    void positionCallback(const Position& incoming_state);
    bool verifyMotorPositions();

    bool verifySensorData();

private:
    Position recent_states[10];
};


#endif