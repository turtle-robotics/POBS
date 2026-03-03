#ifndef COMMON_TYPES_HPP
#define COMMON_TYPES_HPP

#include <vector>

struct Position {
    double x;
    double y;
    double z;
};

struct Orientation {
    double roll;
    double pitch;
    double yaw;
};

struct State {
    Position position;
    Orientation orientation;
    double velocity;
};


#endif