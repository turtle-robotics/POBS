#ifndef STATE_NODE_HPP
#define STATE_NODE_HPP

#include "common_types.hpp"
#include "kalman_filter.hpp"

class StateNode {
public:
    //Starts the state publisher node on the ros network
    StateNode(KalmanFilter* kalman, State* goal_state);

    //this is the overwrite to publish the state
    void publish(const State& state);
    
    //this is the call back from getting to goal state
    void goalStateCallback(const State& incoming_state);
    
    
private:
    //Stores the current state this is read only fron the state estimator
    KalmanFilter* kalman;
};

#endif