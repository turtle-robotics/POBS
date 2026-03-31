#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include "common_types.hpp"
#include <Eigen/Dense>
#include <string>

//This is the struct that should contain any parameters needed for the filter
struct FilterParams {
    double depth_noise;        // depth measurement variance (m^2)
    double gps_noise;          // GPS measurement variance (m^2)
    double imu_accel_noise_x;  // accelerometer variance (m/s^2)^2
    double imu_accel_noise_y;
    double imu_accel_noise_z;
    double imu_gyro_noise;     // gyroscope variance (rad/s)^2
};

//Mantains a estimate of the current robot state.
//
// Internal state vector x_ (10x1):
//   [pos_x, pos_y, pos_z,  vel_x, vel_y, vel_z,  q_l, q_i, q_j, q_k]
//    0      1      2        3      4      5        6    7    8    9
//
// The IMU gives measurements in body frame. Orientation (q) is used to
// rotate body-frame acceleration into world frame before integration,
// so orientation error couples directly into position/velocity error.
class KalmanFilter {
public:
    //This loads the paramenters for the filter and starts listening
    KalmanFilter(std::string file_name);
    //Starts with default params
    KalmanFilter();

    //Helper function that allows the manual loading of a file
    void loadParams(std::string file_name);

    //Takes in a GPS measurement and preforms a observation modifing the state, returns the resulting state
    State measurementGPS(const Position& gps_data);

    //Takes in a depth pressure measurement and preforms a observation modifing the state, returns the resulting state
    State measurementDepth(double pressure);

    //EKF prediction step using body-frame IMU data.
    //accel_body: 3-axis accelerometer reading in body frame (m/s^2)
    //gyro_body:  3-axis gyroscope angular velocity in body frame (rad/s)
    //dt:         time since last IMU sample (s)
    State measurementIMU(const Vector3<double>& accel_body,
                         const Vector3<double>& gyro_body,
                         double dt);

    //Takes in an action of the model
    State actionChange(StateTransition transition);

    //Returns the current position
    Position estimatePosition();
    //Returns the current state
    State estimateState();

private:
    FilterParams filter_params;

    // State vector and covariance
    Eigen::VectorXd x_;   // 10x1
    Eigen::MatrixXd P_;   // 10x10

    // Convert internal Eigen state to the public State struct
    State toState() const;

    // Build rotation matrix R (body -> world) from current quaternion (x_[6..9])
    Eigen::Matrix3d rotFromQuat() const;

    // Jacobian of R(q)*a with respect to q  (3x4).
    // Used by the EKF to propagate how orientation uncertainty affects acceleration.
    static Eigen::Matrix<double, 3, 4> rotJacobian(const Eigen::Vector4d& q,
                                                    const Eigen::Vector3d& a);

    long last_update;
    long last_prediction;
};

#endif
