#include "kalman_filter.hpp"
#include "json.hpp"
#include <fstream>
using json = nlohmann::json;

Position KalmanFilter::estimatePosition(){
    return estimated_state.position;
}

State KalmanFilter::estimateState(){
    return estimated_state;
}

//TODO: Implement
KalmanFilter::KalmanFilter(std::string file_name){
    loadParams(file_name);
}

//TODO: Implement
KalmanFilter::KalmanFilter(){

}

//TODO: all added parameters need to be loaded through here
void KalmanFilter::loadParams(std::string file_name){
    std::ifstream f(file_name);
    json data = json::parse(f);
    filter_params.depth_noise=data["depth_noise"];
    filter_params.gps_noise=data["gps_noise"];

}

State KalmanFilter::measurementGPS(const Position& gps_data){

    // residuals
    double x_residual = gps_data.x - estimated_state.position.x;
    double y_residual = gps_data.y - estimated_state.position.y;

    // innovation covariance
    double x_uncertainty = state_error[0][0] + filter_params.gps_noise;
    double y_uncertainty = state_error[1][1] + filter_params.gps_noise;

    // Kalman gains
    double kx = state_error[0][0] / x_uncertainty;
    double ky = state_error[1][1] / y_uncertainty;

    // state update
    estimated_state.position.x += kx * x_residual;
    estimated_state.position.y += ky * y_residual;

    // covariance update
    state_error[0][0] *= (1 - kx);
    state_error[1][1] *= (1 - ky);

    return estimated_state;
}

//TODO: Implement
//Takes depth in meters
State KalmanFilter::measurementDepth(double depth_meters){
    double z_pred = estimated_state.position.z;
    double y = depth_meters - z_pred;

    double uncertianty = state_error[3][3] + filter_params.depth_noise;
    double gain = state_error[3][3] / uncertianty;
    double result = z_pred + gain * y;
    state_error[3][3] = (1-gain)*state_error[3][3];

    estimated_state.position.z = result;

    return estimated_state;
    
}

//TODO: Implement
State KalmanFilter::measurementIMU(const Orientation& imu_data){
    
}

//TODO: Implement
State KalmanFilter::actionChange(StateTransition transition){

}


