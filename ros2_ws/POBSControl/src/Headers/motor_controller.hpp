#ifndef MOTOR_CONTROLLER_HPP
#define MOTOR_CONTROLLER_HPP

#include "common_types.hpp"
#include "parametric_expressions.hpp"
#include <memory>
#include <string>

// PID state for one degree of freedom.
struct PIDState {
    double kp;           // proportional gain
    double ki;           // integral gain
    double kd;           // derivative gain
    double integral;     // accumulated integral
    double prev_error;   // error from previous step (for derivative)
    double max_integral; // anti-windup saturation limit
};

//This takes in the movements and sends the PID signals
class MotorController {
public:
    //Starts up the controller code
    MotorController(State* goal_state, State* current_state);

    //sets the PWMs of the motors from a direct thrust returns false on failure
    bool commandMotor(int motor_id, double thrust);

    //Sets the PWMs to roll the system. change in rad/s
    StateTransition commandRoll(double roll_change);

    //Sets the PWMs to pitch the system. change in rad/s
    StateTransition commandPitch(double pitch_change);

    //Sets the PWMs to yaw the system. yaw_change in rad/s
    StateTransition commandYaw(double yaw_change);

    //Sets the PWMs to move the system forward. change in m/s/s
    StateTransition commandForward(double acceleration);

    //This is the call back that occurs every cycle.
    bool step();

    bool loadParams(std::string file_name);

    // --- Tunable parameters (adjust per physical vehicle) ---

    // Control loop period in seconds. Must match the actual step() call rate.
    double dt;

    // Per-motor thrust saturation limit (N). Scale to your thruster datasheet.
    double max_motor_thrust;

    // PID controllers — tune gains after characterizing vehicle dynamics.
    // Surge/sway PIDs work on body-frame position error (metres).
    // Heave PID works on depth error (metres, +Z is deeper).
    // Roll/pitch/yaw PIDs work on angle error (radians).
    PIDState pid_surge;
    PIDState pid_sway;
    PIDState pid_heave;
    PIDState pid_roll;
    PIDState pid_pitch;
    PIDState pid_yaw;

private:

    // converter
    double umsToDuty(int ums);

    // Run one PID step given the current error; updates integral and prev_error.
    double runPID(PIDState& pid, double error);

    // Convert unit quaternion (l=w, i=x, j=y, k=z) to ZYX Euler angles (rad).
    // Positive roll  = right side down.
    // Positive pitch = nose down.
    // Positive yaw   = clockwise from above (+Z-down frame).
    static void quatToEuler(const Orientation& q,
                            double& roll, double& pitch, double& yaw);

    // Write thrusts[] from a set of DOF commands using MOTOR_MIX,
    // normalize to max_motor_thrust, then compute PWMs.
    void applyMix(double surge, double sway, double heave,
                  double roll,  double pitch, double yaw);

    // Scale thrusts[] so the largest magnitude does not exceed max_motor_thrust.
    void normalizeThrusts();

    double thrusts[6];
    double pwms[6];
    std::shared_ptr<Expression> thrust_to_pwms[6];

    //This is read only in this function modified in the estimator
    State* goal_state;
    //This is read only in this function modified in the estimator
    State* current_state;

    // Motor mixing matrix.  Rows = motors 0-5, columns = DOFs.
    // DOF column order: [surge, sway, heave, roll, pitch, yaw]
    // Populated by the constructor defaults and overridden by loadParams().
    double motor_mix[6][6];

    //Current motor layout in arrays
    // left to right, front to back. I.E back right is 5, back left is 4, front left is 0

};

#endif
