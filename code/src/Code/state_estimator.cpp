#include "state_estimator.hpp"

//TODO: Implement
StateEstimator::StateEstimator(){
    cur_state = new State;
    goal_state = new State;
    kalman = new KalmanFilter("Json/extended_kalman.json");
    node = new StateNode(kalman,goal_state);
    motorcontroller = new MotorController(goal_state,cur_state);
    
}

//Takes in data saves the state as the function runs
void testFromFile(std::string filename){
    
}

//TODO: Implement
void StateEstimator::loadParams(std::string filename){

}    

//TODO: Implement
void StateEstimator::resetPosition(){

}

