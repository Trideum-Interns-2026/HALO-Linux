#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "geolocation/building_risk_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BuildingRiskNode>());
    rclcpp::shutdown();
    return 0;
}
