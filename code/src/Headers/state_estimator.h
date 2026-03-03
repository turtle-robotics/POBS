#ifndef STATE_ESTIMATOR_HPP
#define STATE_ESTIMATOR_HPP

#include "common_types.h"
#include "state_node.h"
#include "motor_controller.h"

int main();

class StateEstimator {
public:
    StateEstimator();

    void init();
    void loadParams();

    void stateCallback(const State& incoming_state);
    void resetPosition();
    void setStateGoal(const State& goal);

    State getCurrentState() const;

private:
    State cur_state;
    State goal_state;
    KalmanFilter kalman;
    StatePublisher pub;
    MotorController motorcontroller;
};

#endif