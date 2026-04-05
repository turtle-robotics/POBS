#include "kalman_filter.hpp"
#include "json.hpp"
#include <fstream>
#include <stdexcept>
using json = nlohmann::json;

// ── Internal helpers ───────────────────────────────────────────────────────

// State layout: [px, py, pz, vx, vy, vz, ql, qi, qj, qk]
State KalmanFilter::toState() const {
    State s{};
    s.position.x    = x_(0);
    s.position.y    = x_(1);
    s.position.z    = x_(2);
    s.velocites.a   = x_(3);
    s.velocites.b   = x_(4);
    s.velocites.c   = x_(5);
    s.orientation.l = x_(6);
    s.orientation.i = x_(7);
    s.orientation.j = x_(8);
    s.orientation.k = x_(9);
    return s;
}

// Build rotation matrix R (body -> world) from quaternion stored at x_[6..9].
// Convention: q = [ql, qi, qj, qk] = [w, x, y, z]
Eigen::Matrix3d KalmanFilter::rotFromQuat() const {
    const double ql = x_(6), qi = x_(7), qj = x_(8), qk = x_(9);
    Eigen::Matrix3d R;
    R << 1 - 2*(qj*qj + qk*qk),     2*(qi*qj - ql*qk),     2*(qi*qk + ql*qj),
           2*(qi*qj + ql*qk), 1 - 2*(qi*qi + qk*qk),     2*(qj*qk - ql*qi),
           2*(qi*qk - ql*qj),     2*(qj*qk + ql*qi), 1 - 2*(qi*qi + qj*qj);
    return R;
}

// Jacobian of R(q)*a with respect to q (3x4).
// Derived analytically from the rotation matrix formula.
// Each column is d(R*a)/d(q_component) for q = [ql, qi, qj, qk].
Eigen::Matrix<double, 3, 4> KalmanFilter::rotJacobian(const Eigen::Vector4d& q,
                                                        const Eigen::Vector3d& a) {
    const double ql = q(0), qi = q(1), qj = q(2), qk = q(3);
    const double ax = a(0), ay = a(1), az = a(2);

    Eigen::Matrix<double, 3, 4> J;

    // Row 0: d(a_world_x)/d[ql, qi, qj, qk]
    J(0, 0) = -2*qk*ay + 2*qj*az;
    J(0, 1) =  2*qj*ay + 2*qk*az;
    J(0, 2) = -4*qj*ax + 2*qi*ay + 2*ql*az;
    J(0, 3) = -4*qk*ax - 2*ql*ay + 2*qi*az;

    // Row 1: d(a_world_y)/d[ql, qi, qj, qk]
    J(1, 0) =  2*qk*ax - 2*qi*az;
    J(1, 1) =  2*qj*ax - 4*qi*ay - 2*ql*az;
    J(1, 2) =  2*qi*ax             + 2*qk*az;
    J(1, 3) =  2*ql*ax - 4*qk*ay + 2*qj*az;

    // Row 2: d(a_world_z)/d[ql, qi, qj, qk]
    J(2, 0) = -2*qj*ax + 2*qi*ay;
    J(2, 1) =  2*qk*ax + 2*ql*ay - 4*qi*az;
    J(2, 2) = -2*ql*ax + 2*qk*ay - 4*qj*az;
    J(2, 3) =  2*qi*ax + 2*qj*ay;

    return J;
}

// ── Constructors ───────────────────────────────────────────────────────────

KalmanFilter::KalmanFilter(std::string file_name) {
    loadParams(file_name);

    x_ = Eigen::VectorXd::Zero(10);
    x_(6) = 1.0;  // identity quaternion: ql=1, qi=qj=qk=0

    P_ = Eigen::MatrixXd::Identity(10, 10);
}

//TODO: Implement
KalmanFilter::KalmanFilter() {}

// ── Parameter loading ──────────────────────────────────────────────────────

void KalmanFilter::loadParams(std::string file_name) {
    std::ifstream f(file_name);
    if (!f.is_open()) {
        throw std::runtime_error("KalmanFilter::loadParams: could not open " + file_name);
    }
    json data = json::parse(f);
    filter_params.depth_noise       = data["depth_noise"];
    filter_params.gps_noise         = data["gps_noise"];
    filter_params.imu_accel_noise_x = data["imu_accel_noise_x"];
    filter_params.imu_accel_noise_y = data["imu_accel_noise_y"];
    filter_params.imu_accel_noise_z = data["imu_accel_noise_z"];
    filter_params.imu_gyro_noise    = data["imu_gyro_noise"];
}

// ── IMU prediction step (EKF) ──────────────────────────────────────────────
//
// The IMU gives measurements in body frame, so we must rotate accel_body
// into world frame using the current orientation estimate before integrating.
// Orientation error therefore propagates into velocity/position error — the
// EKF Jacobian captures this coupling via the rotJacobian block.
//
// State transition:
//   p_new  = p + v*dt + 0.5 * R(q)*a_b * dt^2
//   v_new  = v + R(q)*a_b * dt
//   q_new  = q + 0.5 * Omega(w_b) * q * dt    (then renormalize)
//
// EKF Jacobian F (10x10):
//   F[0:3, 3:6] = I*dt                      (pos ← vel)
//   F[0:3, 6:10] = 0.5*dt^2 * dR(q)*a/dq   (pos ← q via accel rotation)
//   F[3:6, 6:10] = dt * dR(q)*a/dq          (vel ← q via accel rotation)
//   F[6:10,6:10] = I + 0.5*dt * Omega        (q  ← q via gyro kinematics)

State KalmanFilter::measurementIMU(const Vector3<double>& accel_body,
                                    const Vector3<double>& gyro_body,
                                    double dt) {
    // Current quaternion and body-frame inputs as Eigen types
    Eigen::Vector4d q_vec(x_(6), x_(7), x_(8), x_(9));
    Eigen::Vector3d a_b(accel_body.a, accel_body.b, accel_body.c);
    Eigen::Vector3d w_b(gyro_body.a,  gyro_body.b,  gyro_body.c);

    // Rotate body-frame acceleration to world frame
    Eigen::Matrix3d R   = rotFromQuat();
    Eigen::Vector3d a_w = R * a_b;

    // Jacobian of R(q)*a_b w.r.t. q
    Eigen::Matrix<double, 3, 4> Ja = rotJacobian(q_vec, a_b);

    // Gyro kinematics matrix Omega for q_dot = 0.5 * Omega * q
    const double wx = w_b(0), wy = w_b(1), wz = w_b(2);
    Eigen::Matrix4d Omega;
    Omega <<  0,  -wx, -wy, -wz,
             wx,   0,  wz, -wy,
             wy, -wz,   0,  wx,
             wz,  wy, -wx,   0;

    // ── EKF Jacobian F (10x10) ──────────────────────────────────
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(10, 10);
    F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;       // pos ← vel
    F.block<3, 4>(0, 6) = 0.5 * dt * dt * Ja;                     // pos ← q
    F.block<3, 4>(3, 6) = dt * Ja;                                 // vel ← q
    F.block<4, 4>(6, 6) = Eigen::Matrix4d::Identity()
                          + 0.5 * dt * Omega;                      // q  ← q

    // ── Process noise Q (10x10) ──────────────────────────────────
    // Velocity uncertainty driven by accelerometer noise
    // Position uncertainty driven by velocity uncertainty (dt^4/4 term)
    // Quaternion uncertainty driven by gyroscope noise
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(10, 10);
    Q(0, 0) = filter_params.imu_accel_noise_x * 0.25 * dt*dt*dt*dt;
    Q(1, 1) = filter_params.imu_accel_noise_y * 0.25 * dt*dt*dt*dt;
    Q(2, 2) = filter_params.imu_accel_noise_z * 0.25 * dt*dt*dt*dt;
    Q(3, 3) = filter_params.imu_accel_noise_x * dt*dt;
    Q(4, 4) = filter_params.imu_accel_noise_y * dt*dt;
    Q(5, 5) = filter_params.imu_accel_noise_z * dt*dt;
    Q(6, 6) = filter_params.imu_gyro_noise    * 0.25 * dt*dt;
    Q(7, 7) = filter_params.imu_gyro_noise    * 0.25 * dt*dt;
    Q(8, 8) = filter_params.imu_gyro_noise    * 0.25 * dt*dt;
    Q(9, 9) = filter_params.imu_gyro_noise    * 0.25 * dt*dt;

    // ── State prediction ─────────────────────────────────────────
    x_.head(3)    += x_.segment(3, 3) * dt + 0.5 * a_w * dt * dt;  // position
    x_.segment(3, 3) += a_w * dt;                                    // velocity
    x_.tail(4)    += 0.5 * dt * Omega * q_vec;                       // quaternion
    x_.tail(4).normalize();                                           // unit constraint

    // ── Covariance prediction ────────────────────────────────────
    P_ = F * P_ * F.transpose() + Q;

    return toState();
}

// ── GPS measurement update ─────────────────────────────────────────────────

State KalmanFilter::measurementGPSVelocity(const Vector3<double>& gps_data){
    
}

State KalmanFilter::measurementGPSPosition(const Position& gps_data) {
    // H (2x10): observes pos_x, pos_y
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, 10);
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;

    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(2, 2) * filter_params.gps_noise;

    Eigen::VectorXd z(2);
    z(0) = gps_data.x;
    z(1) = gps_data.y;

    Eigen::VectorXd innov = z - H * x_;
    Eigen::MatrixXd S     = H * P_ * H.transpose() + R;
    Eigen::MatrixXd K     = P_ * H.transpose() * S.inverse();

    x_ = x_ + K * innov;
    P_ = (Eigen::MatrixXd::Identity(10, 10) - K * H) * P_;

    // Renormalize quaternion after update
    x_.tail(4).normalize();

    return toState();
}

// ── Depth measurement update ───────────────────────────────────────────────

State KalmanFilter::measurementDepth(double depth_meters) {
    // H (1x10): observes pos_z
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, 10);
    H(0, 2) = 1.0;

    Eigen::MatrixXd R(1, 1);
    R(0, 0) = filter_params.depth_noise;

    Eigen::VectorXd z(1);
    z(0) = depth_meters;

    Eigen::VectorXd innov = z - H * x_;
    Eigen::MatrixXd S     = H * P_ * H.transpose() + R;
    Eigen::MatrixXd K     = P_ * H.transpose() * S.inverse();

    x_ = x_ + K * innov;
    P_ = (Eigen::MatrixXd::Identity(10, 10) - K * H) * P_;

    return toState();
}

double KalmanFilter::errorMagnitude(){
    return std::sqrt(P_.trace());
}

Position KalmanFilter::estimatePosition() { return toState().position; }
State    KalmanFilter::estimateState()    { return toState(); }

//TODO: Implement
State KalmanFilter::actionChange(StateTransition transition) {}
