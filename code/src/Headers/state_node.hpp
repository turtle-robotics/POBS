#ifndef STATE_NODE_HPP
#define STATE_NODE_HPP

#include "common_types.hpp"
#include "kalman_filter.hpp"

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

// Publishes the current estimated state as nav_msgs/Odometry on /pobs/state.
// Subscribes to /pobs/goal_state (geometry_msgs/PoseStamped) and writes the
// received goal into the shared goal_state pointer owned by StateEstimator.
class StateNode : public rclcpp::Node {
public:
    // kalman     — used by StateEstimator to push updated states via publish()
    // goal_state — written here when a new goal arrives over ROS
    StateNode(KalmanFilter* kalman, State* goal_state);

    // Convert state to nav_msgs/Odometry and publish
    void publish(const State& state);

    // Write incoming goal into *goal_state (called from the ROS subscriber)
    void goalStateCallback(const State& incoming_state);

private:
    KalmanFilter* kalman;
    State*        goal_state;   // shared with StateEstimator — write only from callbacks

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr         state_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
};

#endif
