#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "Tracking/target_kalman_filter.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TargetKalmanFilterNode>());
    rclcpp::shutdown();
    return 0;
}

/*
cd /workspace
source /opt/ros/foxy/setup.bash
cmake -S . -B build
cmake --build build
*/