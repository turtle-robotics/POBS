"""
Kalman Filter Test Data Generator
==================================
Generates synthetic sensor data (IMU, Depth, GPS) under optimal conditions:
  - Linear, independent (Gaussian) measurement noise
  - True state follows a smooth trajectory

Output format:
  1. Noise matrices (R per sensor, Q for process)
  2. Sensor readings: sensor_name: data
     - 100 IMU + 100 Depth readings per 1 GPS reading
"""
import math

import numpy as np

# ─────────────────────────────────────────────
# CONFIGURATION
# ─────────────────────────────────────────────

np.random.seed(42)

NUM_GPS_POINTS   = 20       # total GPS measurements
DT               = 0.01     # time step between IMU/Depth samples (seconds)
GPS_INTERVAL     = 1000      # IMU/Depth samples per GPS sample
DEPTH_INTERVAL   = 3

# ─── True state: [x, y, z, vx, vy, vz] ──────
INITIAL_STATE = np.array([0.0, 0.0, -5.0,   # position (m)  z is depth (negative = below surface)
                           0.0, 0.0, 0.0])   # velocity (m/s) — starts from rest, built up by IMU

# proportional gain — tune this for faster/slower correction
Rotational_gain = 0.2
Acceleration_gain = 1

# ─────────────────────────────────────────────
# NOISE PARAMETERS  (1-sigma std deviations)
# ─────────────────────────────────────────────

# IMU measures linear acceleration [ax, ay, az]  (m/s²)
IMU_STD  = np.array([0.05, 0.05, 0.05])
# Gyroscope measures angular velocity [wx, wy, wz]  (rad/s)
GYRO_STD = np.array([0.01, 0.01, 0.01])

# Depth sensor measures z position (m)
DEPTH_STD = np.array([0.04])

# GPS measures x, y position (m)  — lower update rate, higher positional noise
GPS_STD = np.array([1.50, 1.50])

# Process noise std for state propagation [x, y, z, vx, vy, vz]
PROCESS_STD = np.array([0.001, 0.001, 0.001,
                         0.010, 0.010, 0.010])

# ─────────────────────────────────────────────
# NOISE MATRICES
# ─────────────────────────────────────────────

R_imu   = np.diag(IMU_STD   ** 2)   # 3×3
R_depth = np.diag(DEPTH_STD ** 2)   # 1×1
R_gps   = np.diag(GPS_STD   ** 2)   # 2×2
Q       = np.diag(PROCESS_STD ** 2) # 6×6 process noise covariance

# ─────────────────────────────────────────────
# SIMPLE STATE PROPAGATION
# ─────────────────────────────────────────────

def true_angular_velocity(t: float, rot) -> np.ndarray:
    """Should take the current value and slowly correct it to 0"""

    # Extract axis-angle from rotation matrix via the skew-symmetric part
    # R = I + sin(θ)·K + (1-cos(θ))·K²  →  (R - Rᵀ)/2 = sin(θ)·K
    skew = (rot - rot.T) / 2.0
    sin_theta = np.array([skew[2, 1], skew[0, 2], skew[1, 0]])

    # sin_theta ≈ axis * sin(angle); negate to drive rotation toward identity
    return -Rotational_gain * sin_theta


def quat_to_rot(q: np.ndarray) -> np.ndarray:
    """Unit quaternion [w, x, y, z] -> 3x3 rotation matrix (body -> world)."""
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z),   2*(x*y - w*z),       2*(x*z + w*y)],
        [    2*(x*y + w*z), 1 - 2*(x*x + z*z),       2*(y*z - w*x)],
        [    2*(x*z - w*y),     2*(y*z + w*x),   1 - 2*(x*x + y*y)],
    ])


def propagate_quat(q: np.ndarray, w_body: np.ndarray, dt: float) -> np.ndarray:
    """Integrate quaternion kinematics: q_dot = 0.5 * Omega(w) * q."""
    wx, wy, wz = w_body
    Omega = 0.5 * np.array([
        [  0, -wx, -wy, -wz],
        [ wx,   0,  wz, -wy],
        [ wy, -wz,   0,  wx],
        [ wz,  wy, -wx,   0],
    ])
    q_new = q + Omega @ q * dt
    return q_new / np.linalg.norm(q_new)


def true_acceleration(cur_vel: np.ndarray, goal_vel: np.ndarray) -> np.ndarray:
    """Returns an acceleration that drives cur_vel toward goal_vel."""
    return Acceleration_gain * (goal_vel - cur_vel)


def find_goal_vel(t:float):
    return np.array([5*(-abs(math.sin(t/math.pi/100))+.5*math.sin(2*t/math.pi/100)),3,3])

def propagate(state: np.ndarray, dt: float, accel: np.ndarray) -> np.ndarray:
    """Constant-velocity model + explicit acceleration input."""
    F = np.eye(6)
    F[0, 3] = dt
    F[1, 4] = dt
    F[2, 5] = dt
    next_state = F @ state + np.random.multivariate_normal(np.zeros(6), Q * dt)
    # Apply true acceleration to velocity
    next_state[3] += accel[0] * dt
    next_state[4] += accel[1] * dt
    next_state[5] += accel[2] * dt
    return next_state


# ─────────────────────────────────────────────
# MEASUREMENT MODELS
# ─────────────────────────────────────────────

def measure_imu(a_world: np.ndarray, w_true: np.ndarray,
                q_true: np.ndarray) -> np.ndarray:
    """
    IMU reports acceleration and angular rate in body frame.
    Rotate world-frame accel into body frame, then add independent noise.
    Returns [ax_body, ay_body, az_body, wx, wy, wz]  shape (6,)
    """
    R        = quat_to_rot(q_true)          # body -> world
    a_body   = R.T @ a_world                # world -> body
    a_meas   = a_body + np.random.normal(0, IMU_STD)
    w_meas   = w_true + np.random.normal(0, GYRO_STD)
    return np.concatenate([a_meas, w_meas]) # shape (6,)


def measure_depth(state: np.ndarray) -> np.ndarray:
    """Depth sensor: observe z position."""
    true_z = state[2]
    noise  = np.random.normal(0, DEPTH_STD)
    return np.array([true_z]) + noise         # shape (1,)


def measure_gps(state: np.ndarray) -> np.ndarray:
    """GPS: observe x, y position."""
    true_xy = state[:2]
    noise   = np.random.normal(0, GPS_STD)
    return true_xy + noise                    # shape (2,)


# ─────────────────────────────────────────────
# HELPERS
# ─────────────────────────────────────────────

def fmt_matrix(name: str, M: np.ndarray) -> str:
    rows = []
    for row in M:
        rows.append("  [" + ", ".join(f"{v:10.6f}" for v in row) + "]")
    return f"{name} ({M.shape[0]}×{M.shape[1]}):\n" + "\n".join(rows)


def fmt_vector(v: np.ndarray) -> str:
    return "[" + ", ".join(f"{x:10.6f}" for x in v) + "]"


# ─────────────────────────────────────────────
# MAIN GENERATION
# ─────────────────────────────────────────────

def generate():
    with open("generatedTest.dat","w") as file:
        file.write("=" * 60)
        file.write("\nKALMAN FILTER TEST DATA\n")
        file.write("Optimal scenario: linear, independent Gaussian noise\n")
        file.write("=" * 60)

        # ── Noise matrices ────────────────────────
        file.write("\n--- NOISE MATRICES ---\n")
        file.write(fmt_matrix("R_imu   (IMU measurement noise covariance)\n", R_imu))
        file.write("")
        file.write(fmt_matrix("R_depth (Depth measurement noise covariance)\n", R_depth))
        file.write("")
        file.write(fmt_matrix("R_gps   (GPS measurement noise covariance)\n", R_gps))
        file.write("")
        file.write(fmt_matrix("Q       (Process noise covariance)\n", Q))
        file.write("")

        # ── Sensor data ───────────────────────────
        file.write("--- SENSOR DATA ---\n")
        file.write(f"Format: sensor_name: [values]\n")
        file.write(f"Ratio:  100 IMU + 100 Depth per 1 GPS\n")

        state  = INITIAL_STATE.copy()
        q_true = np.array([1.0, 0.0, 0.0, 0.0])  # identity quaternion [w,x,y,z]
        t      = 0.0

        for gps_idx in range(NUM_GPS_POINTS):
            file.write(f"# GPS epoch {gps_idx + 1}  (t = {t:.3f} – {t + GPS_INTERVAL * DT:.3f} s)\n")

            # 100 IMU + 50 Depth samples before each GPS fix
            for i in range(GPS_INTERVAL):
                for j in range(DEPTH_INTERVAL):
                    accel  = true_acceleration(state[3:],find_goal_vel(t))
                    w_true = true_angular_velocity(t,quat_to_rot(q_true))
                    state  = propagate(state, DT, accel)
                    q_true = propagate_quat(q_true, w_true, DT)
                    t     += DT

                    imu_meas = measure_imu(accel, w_true, q_true)
                    file.write(f"IMU:   {fmt_vector(imu_meas)}\n")

                depth_meas = measure_depth(state)
                file.write(f"DEPTH: {fmt_vector(depth_meas)}\n")

            # One GPS measurement
            gps_meas = measure_gps(state)
            file.write(f"GPS:   {fmt_vector(gps_meas)}\n")
            file.write("")


if __name__ == "__main__":
    generate()