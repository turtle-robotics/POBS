#ifndef SENSOR_READER_HPP
#define SENSOR_READER_HPP

#include "common_types.h"

int main();

class SensorNode {
public:
    SensorNode();

    double readPressure();
    Orientation readIMU();

    State getGoalState();
    State getStateError(const State& current, const State& goal);
};

#endif