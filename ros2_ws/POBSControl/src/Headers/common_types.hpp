#ifndef COMMON_TYPES_HPP
#define COMMON_TYPES_HPP

#include <vector>
#include <functional>
#include "parametric_expressions.hpp"

//used to store global positions
struct Position
{
    double x;
    double y;
    double z;
};

//used to store the current orientation as a quaternion
struct Orientation
{
    double l;
    double i;
    double j;
    double k;
};

//Arbitrary 3 Vector
template <typename T>
struct Vector3{
    T a;
    T b;
    T c;
};

//Used to store the input data from a IMU
struct IMUData
{
    Orientation gyro;
    Vector3<double> accel;
    Vector3<double> mag;
};

//The current bias of the IMU
struct IMUBias{
    Orientation gyro;
    Vector3<double> accel;
};


//The current state of the robot
struct State
{
    //This is a global position relative to the initial position. Except the z is relative to the surface, even if the sub starts above that.
    Position position;
    //This is a relative orientation to the orientation at calibration
    Orientation orientation;
    //This is relative velocities to the current orientation
    Vector3<double> velocites;
    IMUBias imu_bias;
};

struct StateTransition{

    double* transition;
    double** error;
};

#endif