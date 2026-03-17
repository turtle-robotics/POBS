#ifndef COMMON_TYPES_HPP
#define COMMON_TYPES_HPP

#include <vector>
#include <functional>

//used to store global positions
struct Position
{
    double x;
    double y;
    double z;
};

//used to store the current orientation
struct Orientation
{
    double roll;
    double pitch;
    double yaw;
};

//used to store arbitrary 3 vectors
struct Double3
{
    double a;
    double b;
    double c;
};

//Used to store the input data from a IMU
struct IMUData
{
    Orientation gyro;
    Double3 accel;
    Double3 mag;
};

//The current state of the robot
struct State
{
    Position position;
    Orientation orientation;
    Double3 velocites;
};



#endif