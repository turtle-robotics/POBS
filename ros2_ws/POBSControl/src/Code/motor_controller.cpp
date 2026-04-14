#include "motor_controller.hpp"
#include "json.hpp"
#include <cmath>
#include <cstring>
#include <algorithm>
#include <fstream>

// =============================================================================
// Default motor mixing matrix
// =============================================================================
// Indexed left-to-right, front-to-back (0 = front-left, 5 = back-right).
//
// Body frame convention:
//   +X  forward  (surge)
//   +Y  right    (sway)
//   +Z  up     (heave)
//   Positive roll  = right side down
//   Positive pitch = nose down
//   Positive yaw   = clockwise from above
//
//   Motor 0 — Front-Left,  vertical,   thrusts −Z (downward lift)
//   Motor 1 — Front-Right, vertical,   thrusts −Z (downward lift)
//   Motor 2 — Mid-Left,    vertical,   thrusts −Z (downward lift)
//   Motor 3 — Mid-Right,   vertical,   thrusts −Z (downward lift)
//   Motor 4 — Back-Left,   horizontal, thrusts +X (forward thrust)
//   Motor 5 — Back-Right,  horizontal, thrusts +X (forward thrust)
//
// Heave coefficient is −1 for vertical motors because they thrust downward (-Z);
//   a positive heave command (rise) reduces their lift.
//
// Pitch authority comes from the fore/aft lever arm of motors 0 & 1 only;
//   mid motors (2 & 3) sit near the CG and contribute negligible pitch torque.
//
// LIMITATIONS:
//   Sway  — no angled thrusters; sway authority is zero.
//   Pitch — limited to front vertical motors only.
//   Override via loadParams() to match any hardware changes.
//
// DOF column order: [surge, sway, heave, roll, pitch, yaw]
// =============================================================================
static const double DEFAULT_MOTOR_MIX[6][6] = {
    //  surge   sway  heave   roll  pitch   yaw
    {   0.0,   0.0,  -1.0,  +1.0,  +1.0,  0.0 }, // 0: Front-Left  (vertical)
    {   0.0,   0.0,  -1.0,  -1.0,  +1.0,  0.0 }, // 1: Front-Right (vertical)
    {   0.0,   0.0,  -1.0,  +1.0,   0.0,  0.0 }, // 2: Mid-Left    (vertical)
    {   0.0,   0.0,  -1.0,  -1.0,   0.0,  0.0 }, // 3: Mid-Right   (vertical)
    {  +1.0,   0.0,   0.0,   0.0,   0.0, +1.0 }, // 4: Back-Left   (horizontal)
    {  +1.0,   0.0,   0.0,   0.0,   0.0, -1.0 }, // 5: Back-Right  (horizontal)
};

// =============================================================================
// Construction
// =============================================================================

static PIDState makePID(double kp, double ki, double kd,
                         double max_integral = 5.0) {
    PIDState s;
    s.kp = kp;  s.ki = ki;  s.kd = kd;
    s.integral   = 0.0;
    s.prev_error = 0.0;
    s.max_integral = max_integral;
    return s;
}

MotorController::MotorController(State* goal_state, State* current_state)
    : goal_state(goal_state), current_state(current_state),
      dt(0.05),              // 20 Hz — match to actual step() call rate
      max_motor_thrust(10.0) // Newtons — scale to actual thruster spec
{
    //               kp     ki     kd    max_integral
    pid_surge = makePID(1.0,  0.10,  0.30,  5.0);
    pid_sway  = makePID(1.0,  0.10,  0.30,  5.0);
    pid_heave = makePID(1.0,  0.10,  0.50,  5.0);
    pid_roll  = makePID(2.0,  0.05,  1.00,  3.0);
    pid_pitch = makePID(2.0,  0.05,  1.00,  3.0);
    pid_yaw   = makePID(1.5,  0.10,  0.80,  3.0);

    std::memcpy(motor_mix, DEFAULT_MOTOR_MIX, sizeof(motor_mix));
    std::memset(thrusts, 0, sizeof(thrusts));
    std::memset(pwms,    0, sizeof(pwms));
}

// =============================================================================
// commandMotor — hardware-level implementation left to the hardware layer
// =============================================================================
bool MotorController::commandMotor(int motor_id, double thrust) {
    // TODO: implement hardware PWM output
    return false;
}

// =============================================================================
// PID helpers
// =============================================================================

double MotorController::runPID(PIDState& pid, double error) {
    pid.integral += error * dt;
    // Anti-windup clamp
    if (pid.integral >  pid.max_integral) pid.integral =  pid.max_integral;
    if (pid.integral < -pid.max_integral) pid.integral = -pid.max_integral;

    double derivative  = (error - pid.prev_error) / dt;
    pid.prev_error     = error;

    return pid.kp * error + pid.ki * pid.integral + pid.kd * derivative;
}

// Converts a unit quaternion (l=w, i=x, j=y, k=z) to ZYX Euler angles (rad).
void MotorController::quatToEuler(const Orientation& q,
                                   double& roll, double& pitch, double& yaw) {
    double w = q.l, x = q.i, y = q.j, z = q.k;

    // Roll (rotation about X)
    double sinr_cosp = 2.0 * (w*x + y*z);
    double cosr_cosp = 1.0 - 2.0 * (x*x + y*y);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (rotation about Y) — clamp at gimbal-lock boundary
    double sinp = 2.0 * (w*y - z*x);
    if (std::abs(sinp) >= 1.0)
        pitch = std::copysign(M_PI / 2.0, sinp);
    else
        pitch = std::asin(sinp);

    // Yaw (rotation about Z)
    double siny_cosp = 2.0 * (w*z + x*y);
    double cosy_cosp = 1.0 - 2.0 * (y*y + z*z);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}
// Rotates a world-frame XY position error into body-frame surge/sway.
// Uses the top two rows of Rᵀ where R is the body-to-world rotation.
static void worldToBody(const Orientation& q,
                         double wx, double wy,
                         double& surge, double& sway) {
    double w = q.l, x = q.i, y = q.j, z = q.k;
    surge = (1.0 - 2.0*(y*y + z*z)) * wx + 2.0*(x*y + w*z) * wy;
    sway  =        2.0*(x*y - w*z)  * wx + (1.0 - 2.0*(x*x + z*z)) * wy;
}

// Wraps an angle (radians) to (−π, π].
static double wrapAngle(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

// =============================================================================
// applyMix — converts DOF commands to per-motor thrusts and PWMs
// =============================================================================

void MotorController::applyMix(double surge, double sway, double heave,
                                double roll,  double pitch, double yaw) {
    const double dofs[6] = { surge, sway, heave, roll, pitch, yaw };
    for (int m = 0; m < 6; m++) {
        thrusts[m] = 0.0;
        for (int d = 0; d < 6; d++)
            thrusts[m] += motor_mix[m][d] * dofs[d];
    }
    normalizeThrusts();
    for (int m = 0; m < 6; m++)
        pwms[m] = thrust_to_pwms[m] ? thrust_to_pwms[m]->evaluate(thrusts[m]) : 0.0;
}

// Scales all thrusts proportionally so no motor exceeds max_motor_thrust.
// Preserves the relative thrust ratios (direction of intended motion).
void MotorController::normalizeThrusts() {
    double max_abs = 0.0;
    for (int i = 0; i < 6; i++)
        max_abs = std::max(max_abs, std::abs(thrusts[i]));
    if (max_abs > max_motor_thrust) {
        double scale = max_motor_thrust / max_abs;
        for (int i = 0; i < 6; i++)
            thrusts[i] *= scale;
    }
}

// =============================================================================
// Direct command helpers — bypass the PID for manual override or feedforward.
// The argument is a normalised command in the same units as the PID output.
// =============================================================================

// roll_change: desired roll torque (positive = right side down)
StateTransition MotorController::commandRoll(double roll_change) {
    applyMix(0.0, 0.0, 0.0, roll_change, 0.0, 0.0);
    return { nullptr, nullptr };
}

// pitch_change: desired pitch torque (positive = nose down)
StateTransition MotorController::commandPitch(double pitch_change) {
    applyMix(0.0, 0.0, 0.0, 0.0, pitch_change, 0.0);
    return { nullptr, nullptr };
}

// yaw_change: desired yaw torque (positive = clockwise from above)
StateTransition MotorController::commandYaw(double yaw_change) {
    applyMix(0.0, 0.0, 0.0, 0.0, 0.0, yaw_change);
    return { nullptr, nullptr };
}

// acceleration: desired surge force (positive = forward)
StateTransition MotorController::commandForward(double acceleration) {
    applyMix(acceleration, 0.0, 0.0, 0.0, 0.0, 0.0);
    return { nullptr, nullptr };
}

// =============================================================================
// loadParams — read motor config and PID gains from a JSON file
// =============================================================================
// Any key present in the file overwrites the corresponding in-memory value;
// missing keys leave the current value (constructor defaults) unchanged.
// Returns false if the file cannot be opened or contains malformed JSON.
bool MotorController::loadParams(std::string file_name) {
    std::ifstream f(file_name);
    if (!f.is_open()) return false;

    try {
        nlohmann::json j = nlohmann::json::parse(f, nullptr, true, true);

        if (j.contains("dt"))                dt               = j["dt"].get<double>();
        if (j.contains("max_thrust_limit"))  max_motor_thrust = j["max_thrust_limit"].get<double>();

        // Motor mixing matrix — one row per thruster entry, up to 6
        if (j.contains("Thrusters")) {
            const auto& thrusters = j["Thrusters"];
            if (thrusters.size() > 6) return false;
            for (size_t m = 0; m < thrusters.size(); m++) {
                const auto& mix = thrusters[m]["Motor_Mix"];
                if (mix.size() != 6) return false;
                for (int d = 0; d < 6; d++)
                    motor_mix[m][d] = mix[d].get<double>();
            }
        }

    } catch (...) {
        return false;
    }

    return true;
}

// =============================================================================
// step — full PID control loop, called every cycle at rate 1/dt
// =============================================================================
bool MotorController::step() {
    if (!goal_state || !current_state) return false;

    // 1. Convert quaternions to Euler angles
    double cur_roll, cur_pitch, cur_yaw;
    double goal_roll, goal_pitch, goal_yaw;
    quatToEuler(current_state->orientation, cur_roll,  cur_pitch,  cur_yaw);
    quatToEuler(goal_state->orientation,    goal_roll, goal_pitch, goal_yaw);

    // 2. Orientation errors (wrapped to avoid discontinuity at ±π)
    double roll_error  = wrapAngle(goal_roll  - cur_roll);
    double pitch_error = wrapAngle(goal_pitch - cur_pitch);
    double yaw_error   = wrapAngle(goal_yaw   - cur_yaw);

    // 3. Position error in world frame → rotate into body frame for surge/sway
    double world_dx = goal_state->position.x - current_state->position.x;
    double world_dy = goal_state->position.y - current_state->position.y;
    double surge_error, sway_error;
    worldToBody(current_state->orientation, world_dx, world_dy,
                surge_error, sway_error);

    // Depth: positive Z = deeper, so a positive error means we need to sink
    double heave_error = goal_state->position.z - current_state->position.z;

    // 4. Run PIDs
    double surge_cmd = runPID(pid_surge, surge_error);
    double sway_cmd  = runPID(pid_sway,  sway_error);
    double heave_cmd = runPID(pid_heave, heave_error);
    double roll_cmd  = runPID(pid_roll,  roll_error);
    double pitch_cmd = runPID(pid_pitch, pitch_error);
    double yaw_cmd   = runPID(pid_yaw,   yaw_error);

    // 5. Mix all DOF commands into per-motor thrusts and PWMs
    applyMix(surge_cmd, sway_cmd, heave_cmd,
              roll_cmd,  pitch_cmd, yaw_cmd);

    return true;
}

// =============================================================================
// umsToDuty — microseconds to duty cycle (hardware-specific, fill in later)
// =============================================================================
double MotorController::umsToDuty(int ums) {
    // TODO: implement based on ESC/servo pulse-width specification
    return 0.0;
}
