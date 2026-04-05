#include "state_node.hpp"

// Topic names
static constexpr char STATE_TOPIC[]      = "/pobs/state";
static constexpr char GOAL_TOPIC[]       = "/pobs/goal_state";
static constexpr char WORLD_FRAME[]      = "world";
static constexpr char BODY_FRAME[]       = "pobs";
static constexpr int  QUEUE_DEPTH        = 10;

// ── Constructor ────────────────────────────────────────────────────────────

StateNode::StateNode(KalmanFilter* kalman, State* goal_state)
    : rclcpp::Node("state_node"),
      kalman(kalman),
      goal_state(goal_state)
{
    // Publisher: current estimated state
    state_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        STATE_TOPIC, QUEUE_DEPTH);

    // Subscriber: goal state
    // Converts the incoming PoseStamped message to our internal State type
    // then writes it through the shared goal_state pointer.
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        GOAL_TOPIC, QUEUE_DEPTH,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            State incoming{};

            incoming.position.x = msg->pose.position.x;
            incoming.position.y = msg->pose.position.y;
            incoming.position.z = msg->pose.position.z;

            // ROS quaternion convention is (x, y, z, w)
            // Our Orientation convention is (l, i, j, k) = (w, x, y, z)
            incoming.orientation.l = msg->pose.orientation.w;
            incoming.orientation.i = msg->pose.orientation.x;
            incoming.orientation.j = msg->pose.orientation.y;
            incoming.orientation.k = msg->pose.orientation.z;

            goalStateCallback(incoming);
        }
    );

    RCLCPP_INFO(this->get_logger(),
        "StateNode started — publishing on %s, goal on %s",
        STATE_TOPIC, GOAL_TOPIC);
}

// ── Publish current state ──────────────────────────────────────────────────

void StateNode::publish(const State& state) {
    nav_msgs::msg::Odometry msg;

    msg.header.stamp    = this->get_clock()->now();
    msg.header.frame_id = WORLD_FRAME;
    msg.child_frame_id  = BODY_FRAME;

    // Position
    msg.pose.pose.position.x = state.position.x;
    msg.pose.pose.position.y = state.position.y;
    msg.pose.pose.position.z = state.position.z;

    // Orientation — convert (l, i, j, k) → ROS (x, y, z, w)
    msg.pose.pose.orientation.w = state.orientation.l;
    msg.pose.pose.orientation.x = state.orientation.i;
    msg.pose.pose.orientation.y = state.orientation.j;
    msg.pose.pose.orientation.z = state.orientation.k;

    // Linear velocity in body frame
    msg.twist.twist.linear.x = state.velocites.a;
    msg.twist.twist.linear.y = state.velocites.b;
    msg.twist.twist.linear.z = state.velocites.c;

    state_pub_->publish(msg);
}

// ── Goal state callback ────────────────────────────────────────────────────

void StateNode::goalStateCallback(const State& incoming_state) {
    *goal_state = incoming_state;

    RCLCPP_INFO(this->get_logger(),
        "New goal received — pos(%.2f, %.2f, %.2f)",
        incoming_state.position.x,
        incoming_state.position.y,
        incoming_state.position.z);
}
