#include <memory>
#include <exception>

#include <rclcpp/rclcpp.hpp>

#include "Tracking/track_and_center.hpp"
#include "Tracking/target_kalman_filter.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    try {
        auto kalman_node = std::make_shared<TargetKalmanFilterNode>();
        auto tracking_node = std::make_shared<TrackAndCenterNode>();

        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(kalman_node);
        executor.add_node(tracking_node);
        executor.spin();

        tracking_node->stopOffboard();
        executor.remove_node(tracking_node);
        executor.remove_node(kalman_node);
        tracking_node.reset();
        kalman_node.reset();
    } catch (const std::exception& error) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}