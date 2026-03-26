#include "state_estimator.hpp"

//TODO: Implement
StateEstimator::StateEstimator(){

    cur_state = new State;
    goal_state = new State;
    kalman = new KalmanFilter("Json/extended_kalman.json");
    node = new StateNode(kalman,goal_state);
    motorcontroller = new MotorController(goal_state,cur_state);
}

//TODO: Implement
void StateEstimator::loadParams(){

}    

//TODO: Implement
void StateEstimator::resetPosition(){

}