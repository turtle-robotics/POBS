#ifndef MASTER_PATHER_HPP
#define MASTER_PATHER_HPP

#include "common_types.hpp"
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

// Topic names — must match state_node.cpp
static constexpr char PATH_STATE_TOPIC[] = "/pobs/state";
static constexpr char PATH_GOAL_TOPIC[]  = "/pobs/goal_state";

struct PathParams {
    double arrival_threshold;   // metres — how close counts as "reached"
    double max_turn_radius;     // metres — maximum turning radius for path smoothing
    int    intermediate_points; // number of interpolated waypoints between mission points
    double surface_error_threshold; // max state error before surfacing.
};

// Reads a mission file (JSON array of waypoints) and drives the sub toward each
// one in sequence, publishing the active waypoint as a geometry_msgs/PoseStamped
// on /pobs/goal_state. Advances to the next waypoint once within arrival_threshold
// metres of the current goal (Euclidean, position only).
class MasterPather : public rclcpp::Node {
public:
    // Starts the ROS2 node and loads the mission from file_name
    MasterPather(std::string file_name);

    // Helper to (re)load a mission file at runtime
    bool loadParams(std::string file_name);

    bool loadMission(std::string file_name);

    // Convert the current goal_state to PoseStamped and publish on /pobs/goal_state
    void publishGoalState();

private:
    // Current goal being navigated to
    State goal_state;

    // Distance to current goal, updated on every state message
    double state_error_mag;



    // Loaded waypoints and which one is active
    std::vector<State> waypoints_;
    size_t             waypoint_idx_;

    // Path planning parameters
    PathParams path_params_;

    // Most-recent position estimate from the state estimator
    Position cur_position_;

    bool is_surfacing;

    // ROS2 interfaces
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr    goal_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr         state_sub_;
    rclcpp::TimerBase::SharedPtr                                     publish_timer_;

    // Callback — updates cur_position_ and advances waypoint when close enough
    void stateCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // Euclidean distance between two positions
    static double positionDistance(const Position& a, const Position& b);

    // RMS variance magnitude from an Odometry covariance matrix.
    // The Odometry pose covariance is a row-major 6x6 matrix [x,y,z,roll,pitch,yaw].
    // Returns sqrt(trace) — the combined standard deviation across all 6 dimensions.
    static double odometryVarianceMagnitude(const nav_msgs::msg::Odometry& odom);
};

#endif