#ifndef STATE_NODE_HPP
#define STATE_NODE_HPP

#include "common_types.hpp"
#include "kalman_filter.hpp"

class StateNode {
public:
    StateNode(KalmanFilter* kalman, State* goal_state);

    void publish(const State& state);
    void subscribe();
    
    //this is the call back from getting to goal state
    void goalStateCallback(const State& incoming_state);
    
    
private:
    //Stores the current state this is read only fron the state estimator
    KalmanFilter* kalman;
};

#endif