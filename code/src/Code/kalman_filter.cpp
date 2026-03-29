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

KalmanFilter::KalmanFilter(std::string file_name){
    loadParams(file_name);

    //Initalize variables
    state_error=new double*[16];
    for(int i =0;i<16;i++){
        state_error[i]=new double[16];
    }
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
    filter_params.imu_noise_i=data["imu_noise_i"];
    filter_params.imu_noise_l=data["imu_noise_l"];
    filter_params.imu_noise_j=data["imu_noise_j"];
    filter_params.imu_noise_k=data["imu_noise_k"];
}

//TODO: implement the non idependent error
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

//TODO: implement the non idependent error
//Takes depth in meters
State KalmanFilter::measurementDepth(double depth_meters){
    double z_pred = estimated_state.position.z;
    double y = depth_meters - z_pred;

    double uncertianty = state_error[2][2] + filter_params.depth_noise;
    double gain = state_error[2][2] / uncertianty;
    double result = z_pred + gain * y;
    state_error[2][2] = (1-gain)*state_error[2][2];

    estimated_state.position.z = result;

    return estimated_state;
    
}

//TODO: implement the non idependent error
State KalmanFilter::measurementIMU(const Orientation& imu_data) {
    
    // --- Current estimated quaternion ---
    double q_l = estimated_state.orientation.l;
    double q_i = estimated_state.orientation.i;
    double q_j = estimated_state.orientation.j;
    double q_k = estimated_state.orientation.k;

    // --- Compute innovation via quaternion difference ---
    // q_error = q_measured * q_estimated_conjugate
    // Conjugate of estimated: (l, -i, -j, -k)
    double err_l = imu_data.l*q_l + imu_data.i*q_i + imu_data.j*q_j + imu_data.k*q_k;
    double err_i = imu_data.i*q_l - imu_data.l*q_i + imu_data.k*q_j - imu_data.j*q_k;
    double err_j = imu_data.j*q_l - imu_data.k*q_i - imu_data.l*q_j + imu_data.i*q_k;
    double err_k = imu_data.k*q_l + imu_data.j*q_i - imu_data.i*q_j - imu_data.l*q_k;

    // Ensure shortest-path correction (flip if scalar part is negative)
    if (err_l < 0.0) {
        err_l = -err_l;
        err_i = -err_i;
        err_j = -err_j;
        err_k = -err_k;
    }

    // --- Kalman gains for each quaternion component (indices 4-7) ---
    double uncertainty_l = state_error[4][4] + filter_params.imu_noise_l;
    double uncertainty_i = state_error[5][5] + filter_params.imu_noise_i;
    double uncertainty_j = state_error[6][6] + filter_params.imu_noise_j;
    double uncertainty_k = state_error[7][7] + filter_params.imu_noise_k;

    double gain_l = state_error[4][4] / uncertainty_l;
    double gain_i = state_error[5][5] / uncertainty_i;
    double gain_j = state_error[6][6] / uncertainty_j;
    double gain_k = state_error[7][7] / uncertainty_k;

    // --- State update ---
    // Nudge each component toward the measurement by the innovation scaled by gain
    estimated_state.orientation.l += gain_l * err_l;
    estimated_state.orientation.i += gain_i * err_i;
    estimated_state.orientation.j += gain_j * err_j;
    estimated_state.orientation.k += gain_k * err_k;

    // --- Re-normalize the quaternion (CRITICAL) ---
    double norm = std::sqrt(
        estimated_state.orientation.l * estimated_state.orientation.l +
        estimated_state.orientation.i * estimated_state.orientation.i +
        estimated_state.orientation.j * estimated_state.orientation.j +
        estimated_state.orientation.k * estimated_state.orientation.k
    );
    estimated_state.orientation.l /= norm;
    estimated_state.orientation.i /= norm;
    estimated_state.orientation.j /= norm;
    estimated_state.orientation.k /= norm;

    // --- Covariance update ---
    state_error[4][4] = (1 - gain_l) * state_error[4][4];
    state_error[5][5] = (1 - gain_i) * state_error[5][5];
    state_error[6][6] = (1 - gain_j) * state_error[6][6];
    state_error[7][7] = (1 - gain_k) * state_error[7][7];

    return estimated_state;
}

//TODO: Implement
State KalmanFilter::actionChange(StateTransition transition){

}


