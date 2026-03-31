#ifndef MASTER_PATHER_HPP
#define MASTER_PATHER_HPP

#include "common_types.hpp"
#include <string>

//This is what makes the decisions of what the direction will be
class MasterPather {
public:
    //Starts up the ros node and begins publishing
    MasterPather(std::string file_name);

    //Helper function to allow manual reading from a json
    bool loadParams(std::string file_name);

    //Sends the goal state over the ros network
    void publishGoalState();

private:
    State goal_state;
};

#endif