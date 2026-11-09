#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "geolocation/geolocation_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GeolocationNode>());
    rclcpp::shutdown();
    return 0;
}
