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

import numpy as np

# ─────────────────────────────────────────────
# CONFIGURATION
# ─────────────────────────────────────────────

np.random.seed(42)

NUM_GPS_POINTS   = 10       # total GPS measurements
DT               = 0.01     # time step between IMU/Depth samples (seconds)
GPS_INTERVAL     = 100      # IMU/Depth samples per GPS sample
DEPTH_INTERVAL   = 2

# ─── True state: [x, y, z, vx, vy, vz] ──────
# Simple constant-velocity trajectory with gentle acceleration
INITIAL_STATE = np.array([0.0, 0.0, -5.0,   # position (m)  z is depth (negative = below surface)
                           0.5, 0.2, 0.0])   # velocity (m/s)

# ─────────────────────────────────────────────
# NOISE PARAMETERS  (1-sigma std deviations)
# ─────────────────────────────────────────────

# IMU measures linear acceleration [ax, ay, az]  (m/s²)
IMU_STD = np.array([0.05, 0.05, 0.05])

# Depth sensor measures z position (m)
DEPTH_STD = np.array([0.10])

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

def propagate(state: np.ndarray, dt: float) -> np.ndarray:
    """Constant-velocity model: x_{k+1} = F * x_k"""
    F = np.eye(6)
    F[0, 3] = dt
    F[1, 4] = dt
    F[2, 5] = dt
    return F @ state + np.random.multivariate_normal(np.zeros(6), Q * dt)


# ─────────────────────────────────────────────
# MEASUREMENT MODELS
# ─────────────────────────────────────────────

def measure_imu(state: np.ndarray) -> np.ndarray:
    """
    IMU reports acceleration.  In an optimal scenario the true acceleration
    is 0 (constant-velocity model), so each reading is pure noise centred
    on zero plus any modelled acceleration.
    """
    true_accel = np.zeros(3)                  # constant-velocity → zero accel
    noise = np.random.normal(0, IMU_STD)
    return true_accel + noise                 # shape (3,)


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

        state = INITIAL_STATE.copy()
        t     = 0.0

        for gps_idx in range(NUM_GPS_POINTS):
            file.write(f"# GPS epoch {gps_idx + 1}  (t = {t:.3f} – {t + GPS_INTERVAL * DT:.3f} s)\n")

            # 100 IMU + 100 Depth samples before each GPS fix
            for i in range(GPS_INTERVAL):
                for j in range(DEPTH_INTERVAL):
                    state = propagate(state, DT)
                    t    += DT

                    imu_meas   = measure_imu(state)
                    file.write(f"IMU:   {fmt_vector(imu_meas)}\n")

                depth_meas = measure_depth(state)
                file.write(f"DEPTH: {fmt_vector(depth_meas)}\n")

            # One GPS measurement
            gps_meas = measure_gps(state)
            file.write(f"GPS:   {fmt_vector(gps_meas)}\n")
            file.write("")


if __name__ == "__main__":
    generate()