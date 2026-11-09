#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "tracking/tracking_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrackingNode>());
    rclcpp::shutdown();
    return 0;
}
