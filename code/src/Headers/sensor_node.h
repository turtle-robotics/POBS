#ifndef SENSOR_NODE_HPP
#define SENSOR_NODE_HPP

#include "common_types.h"


struct SensorParams {
};


class SensorNode {
public:
    SensorNode();

    bool loadParams();
    void writeCamera();
    void writeSonar();

    double readPressure();
    Orientation readIMU();

private:
    SensorParams params;
};

#endif