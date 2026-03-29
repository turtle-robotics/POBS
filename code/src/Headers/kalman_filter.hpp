#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include "common_types.hpp"
#include <string>

//This is the struct that should contain any parameters needed for the filter
struct FilterParams {
    double depth_noise;//in m
    double gps_noise;//in m
    double imu_noise_l;
    double imu_noise_i;
    double imu_noise_j;
    double imu_noise_k;
};

//Mantains a estimate of the current robot state
//The current plan is a kalman filter though this might work better with a particle or other filter. 
//This file should implement a kalman we can create others and swap them out if needed.
class KalmanFilter {
public:
    //This loads the paramenters for the filter and starts listening
    KalmanFilter(std::string file_name);
    //Starts with default params
    KalmanFilter();

    //Helper function that allows the manual loading of a file   
    void loadParams(std::string file_name);

    //Takes in a GPS measurement and preforms a observation modifing the state, returns the resulting state
    //There should be heavy weight placed on the gps as it has an absolute accuracy when surfaced
    State measurementGPS(const Position& gps_data);

    //Takes in a depth pressure measurement and preforms a observation modifing the state, returns the resulting state
    //There should be heavy weight placed on the depth as it has an absolute accuracy
    State measurementDepth(double pressure);

    //Takes in a IMU measurement and preforms a observation modifing the state, returns the resulting state
    State measurementIMU(const Orientation& imu_data);

    //Takes in an action of the model
    //We need to do a lot of modeling of the system however that is not the goal of this function.
    //Currently the state in this represnted as the 1 second change in values and the 
    State actionChange(StateTransition transition);

    //Returns the current position
    Position estimatePosition();
    //Returns the current state
    State estimateState();

private:
    FilterParams filter_params;
    State estimated_state;
    double** state_error;
    long last_update;
    long last_prediction;
    
};

#endif