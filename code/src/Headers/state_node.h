#ifndef STATE_NODE_HPP
#define STATE_NODE_HPP

#include "common_types.h"
#include "kalman_filter.h"



class StatePublisher {
public:
    void publish(const State& state);
};

class StateSubscriber {
public:
    void subscribe();
    void stateCallback(const State& state);
    
private:
    KalmanFilter* kalman;
};

#endif