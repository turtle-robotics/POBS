#include "sensor_node.hpp"

static constexpr int QUEUE_DEPTH = 10;

SensorNode::SensorNode(std::string file_name)
    : rclcpp::Node("sensor_node"),
      last_pressure(0.0)
{
    state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        SENSOR_STATE_TOPIC, QUEUE_DEPTH,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            stateCallback(msg);
        });

    RCLCPP_INFO(this->get_logger(),
        "SensorNode started — listening on %s", SENSOR_STATE_TOPIC);
}

// ── State subscriber callback ──────────────────────────────────────────────

void SensorNode::stateCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Position
    state.position.x = msg->pose.pose.position.x;
    state.position.y = msg->pose.pose.position.y;
    state.position.z = msg->pose.pose.position.z;

    // Orientation — ROS (x,y,z,w) → internal (l=w, i=x, j=y, k=z)
    state.orientation.l = msg->pose.pose.orientation.w;
    state.orientation.i = msg->pose.pose.orientation.x;
    state.orientation.j = msg->pose.pose.orientation.y;
    state.orientation.k = msg->pose.pose.orientation.z;

    // Body-frame linear velocity
    state.velocites.a = msg->twist.twist.linear.x;
    state.velocites.b = msg->twist.twist.linear.y;
    state.velocites.c = msg->twist.twist.linear.z;
}

// ── Not yet implemented ────────────────────────────────────────────────────

bool SensorNode::loadParams(std::string file_name) {
    // TODO: Implement
    return false;
}

void SensorNode::writeCamera() {
    // TODO: Implement
}

void SensorNode::writeSonar() {
    // TODO: Implement
}
