#include "state_estimator.hpp"
#include "path_master.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    // StateEstimator owns the StateNode (Kalman filter + current state publisher)
    StateEstimator state;

    // MasterPather publishes goal states and subscribes to current state
    auto pather = std::make_shared<MasterPather>("src/Json/path_goals.json");
    pather->loadParams("src/Json/path_params.json");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(state.getNode());
    executor.add_node(pather);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
