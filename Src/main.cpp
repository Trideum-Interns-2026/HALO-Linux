#include <memory>
#include <exception>

#include <rclcpp/rclcpp.hpp>

#include "Tracking/target_kalman_filter.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    try {
        auto kalman_node = std::make_shared<TargetKalmanFilterNode>();

        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(kalman_node);
        executor.spin();

        executor.remove_node(kalman_node);
        kalman_node.reset();
    } catch (const std::exception& error) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}