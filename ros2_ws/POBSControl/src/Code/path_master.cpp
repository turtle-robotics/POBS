#include "path_master.hpp"
#include "json.hpp"
#include <fstream>
#include <stdexcept>
#include <cmath>
#include <chrono>

using json = nlohmann::json;
using namespace std::chrono_literals;

static constexpr int    QUEUE_DEPTH          = 10;
static constexpr auto   PUBLISH_PERIOD       = 100ms;  // 10 Hz

// Default path params (used until loadParams is called)
static constexpr double DEFAULT_THRESHOLD    = 0.5;   // metres
static constexpr double DEFAULT_TURN_RADIUS  = 1.0;   // metres
static constexpr int    DEFAULT_INTERP_PTS   = 5;

// ── Constructor ────────────────────────────────────────────────────────────

MasterPather::MasterPather(std::string file_name)
    : rclcpp::Node("master_pather"),
      state_error_mag(0.0),
      waypoint_idx_(0),
      path_params_{DEFAULT_THRESHOLD, DEFAULT_TURN_RADIUS, DEFAULT_INTERP_PTS,.1},
      cur_position_{0.0, 0.0, 0.0}
{
    // Publisher: goal state for the sub to navigate toward
    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        PATH_GOAL_TOPIC, QUEUE_DEPTH);

    // Subscriber: current state estimate from the Kalman filter
    state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        PATH_STATE_TOPIC, QUEUE_DEPTH,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            stateCallback(msg);
        });

    // Timer: periodically republish the active goal so late-joining nodes
    // receive it without waiting for a state update to trigger a re-publish
    // publish_timer_ = this->create_wall_timer(
    //     PUBLISH_PERIOD, [this]() { publishGoalState(); });

    if (!loadParams(file_name)) {
        RCLCPP_ERROR(this->get_logger(),
            "MasterPather: failed to load mission file '%s'", file_name.c_str());
    }

    RCLCPP_INFO(this->get_logger(),
        "MasterPather started — %zu waypoints loaded, goal on %s",
        waypoints_.size(), PATH_GOAL_TOPIC);

    is_surfacing=false;
}

// ── Parameter / mission loading ────────────────────────────────────────────

bool MasterPather::loadMission(std::string file_name){
    std::ifstream f(file_name);
    if (!f.is_open()) {
        RCLCPP_ERROR(this->get_logger(),
            "MasterPather::loadParams: could not open '%s'", file_name.c_str());
        return false;
    }

    json data = json::parse(f, nullptr, /*exceptions=*/false);
    if (data.is_discarded()) {
        RCLCPP_ERROR(this->get_logger(),
            "MasterPather::loadParams: JSON parse error in '%s'", file_name.c_str());
        return false;
    }

    if (data.contains("arrival_threshold")) {
        path_params_.arrival_threshold = data["arrival_threshold"];
    }

    waypoints_.clear();
    waypoint_idx_ = 0;

    for (const auto& wp : data["waypoints"]) {
        State s{};
        s.position.x    = wp["position"]["x"];
        s.position.y    = wp["position"]["y"];
        s.position.z    = wp["position"]["z"];
        s.orientation.l = wp["orientation"]["l"];
        s.orientation.i = wp["orientation"]["i"];
        s.orientation.j = wp["orientation"]["j"];
        s.orientation.k = wp["orientation"]["k"];
        waypoints_.push_back(s);
    }

    if (!waypoints_.empty()) {
        goal_state = waypoints_[0];
    }

    return true;
}


bool MasterPather::loadParams(std::string file_name) {
    std::ifstream f(file_name);
    if (!f.is_open()) {
        RCLCPP_ERROR(this->get_logger(),
            "MasterPather::loadParams: could not open '%s'", file_name.c_str());
        return false;
    }

    json data = json::parse(f, nullptr, /*exceptions=*/false);
    if (data.is_discarded()) {
        RCLCPP_ERROR(this->get_logger(),
            "MasterPather::loadParams: JSON parse error in '%s'", file_name.c_str());
        return false;
    }

    if (data.contains("arrival_threshold"))   path_params_.arrival_threshold   = data["arrival_threshold"];
    if (data.contains("max_turn_radius"))     path_params_.max_turn_radius     = data["max_turn_radius"];
    if (data.contains("intermediate_points")) path_params_.intermediate_points = data["intermediate_points"];

    return true;
}

// ── State subscriber callback ──────────────────────────────────────────────

void MasterPather::stateCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    cur_position_.x = msg->pose.pose.position.x;
    cur_position_.y = msg->pose.pose.position.y;
    cur_position_.z = msg->pose.pose.position.z;

    double error_mag = positionDistance(cur_position_, goal_state.position);

    if(odometryVarianceMagnitude(*msg)>path_params_.surface_error_threshold){
        is_surfacing=true;

        RCLCPP_INFO(this->get_logger(),
            "Error threshold reached, surfacing %zu / %zu  "
            "pos(%.2f, %.2f, %.2f)",
            waypoint_idx_, waypoints_.size() - 1,
            goal_state.position.x,
            goal_state.position.y,
            goal_state.position.z);
    }

    // Advance to the next waypoint once we're close enough
    if (error_mag < path_params_.arrival_threshold && waypoint_idx_ + 1 < waypoints_.size()) {
        if(!is_surfacing){
            waypoint_idx_++;
        }
        else{
            is_surfacing=false;//Dosen't stay on the surface for any amount of time.
        }
        goal_state = waypoints_[waypoint_idx_];

        RCLCPP_INFO(this->get_logger(),
            "Waypoint reached — advancing to waypoint %zu / %zu  "
            "pos(%.2f, %.2f, %.2f)",
            waypoint_idx_, waypoints_.size() - 1,
            goal_state.position.x,
            goal_state.position.y,
            goal_state.position.z);
    }
}

// ── Publish goal ───────────────────────────────────────────────────────────

void MasterPather::publishGoalState() {
    if (waypoints_.empty()) {
        return;
    }

    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp    = this->get_clock()->now();
    msg.header.frame_id = "world";

    msg.pose.position.x = goal_state.position.x;
    msg.pose.position.y = goal_state.position.y;
    msg.pose.position.z = goal_state.position.z;

    // Internal Orientation (l, i, j, k) = (w, x, y, z) → ROS (x, y, z, w)
    msg.pose.orientation.w = goal_state.orientation.l;
    msg.pose.orientation.x = goal_state.orientation.i;
    msg.pose.orientation.y = goal_state.orientation.j;
    msg.pose.orientation.z = goal_state.orientation.k;

    goal_pub_->publish(msg);
}

// ── Helpers ────────────────────────────────────────────────────────────────

double MasterPather::positionDistance(const Position& a, const Position& b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    double dz = a.z - b.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

// The pose covariance is a row-major 6x6 matrix indexed as [row*6 + col].
// Diagonal indices 0,7,14,21,28,35 correspond to variances of x,y,z,roll,pitch,yaw.
// Returns sqrt(sum of diagonal) — RMS spread across all 6 pose dimensions.
double MasterPather::odometryVarianceMagnitude(const nav_msgs::msg::Odometry& odom) {
    const auto& cov = odom.pose.covariance;
    double trace = cov[0] + cov[7] + cov[14] + cov[21] + cov[28] + cov[35];
    return std::sqrt(trace);
}