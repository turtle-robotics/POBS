#ifndef SENSOR_NODE_HPP
#define SENSOR_NODE_HPP

#include "common_types.hpp"
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

static constexpr char SENSOR_STATE_TOPIC[] = "/pobs/state";

//Use this to store the needed params for the code
struct SensorParams {
};

//This node subscribes to /pobs/state and stores the latest state estimate
class SensorNode : public rclcpp::Node {
public:
    //Starts up the ros node and subscribes to the state topic
    SensorNode(std::string file_name);

    //Helper function to allow manual reading from a json
    bool loadParams(std::string file_name);
    //Saves camera to a file
    void writeCamera();
    //Saves sonar to a file to a file
    void writeSonar();

    //Called when a new state message arrives — converts and stores it
    void stateCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    //Returns the most recently stored state
    const State& getState() const { return state; }

private:
    SensorParams params;
    State state;
    IMUData last_imu;
    double last_pressure;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_sub_;
};



#endif