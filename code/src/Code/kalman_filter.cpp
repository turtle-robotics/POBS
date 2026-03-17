#include "kalman_filter.hpp"

Position KalmanFilter::estimatePosition(){
    return estimated_state.position;
}

State KalmanFilter::estimateState(){
    return estimated_state;
}

//TODO: Implement
KalmanFilter::KalmanFilter(std::string file_name){

}
//TODO: Implement
KalmanFilter::KalmanFilter(std::string file_name){

}

//TODO: Implement 
void KalmanFilter::loadParams(std::string file_name){

}

//TODO: Implement
State KalmanFilter::measurementGPS(const Position& gps_data){

}

//TODO: Implement
State KalmanFilter::measurementDepth(double pressure){

}

//TODO: Implement
State KalmanFilter::measurementIMU(const Orientation& imu_data){
    
}