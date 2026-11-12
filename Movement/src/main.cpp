#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "movement/heading_freshness.hpp"
#include "movement/mavsdk_vehicle.hpp"
#include "movement/movement_node.hpp"

namespace {
// SITL default; override with --ros-args -p mavsdk_url:=... for HITL / hardware.
constexpr char kDefaultMavsdkUrl[] = "udpin://0.0.0.0:14540";
}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    const auto logger = rclcpp::get_logger("movement");
    int status = 0;
    try {
        // The URL parameter is read on a short-lived node so the vehicle exists
        // before MovementNode, which owns it.
        auto params = std::make_shared<rclcpp::Node>("movement_params");
        const std::string url =
            params->declare_parameter("mavsdk_url", std::string(kDefaultMavsdkUrl));
        const double heading_timeout_s = movement::sanitizeHeadingTimeout(
            params->declare_parameter("heading_timeout_s", movement::kDefaultHeadingTimeoutS));
        params.reset();

        auto node = std::make_shared<movement::MovementNode>(
            std::make_unique<movement::MavsdkVehicle>(url, heading_timeout_s),
            rclcpp::NodeOptions(), url, heading_timeout_s);
        try {
            rclcpp::spin(node);
        } catch (...) {
            node->shutdown();
            throw;
        }
        node->shutdown();  // Ctrl+C: stop the timer, return PX4 to Hold
    } catch (const std::exception& error) {
        RCLCPP_ERROR(logger, "%s", error.what());
        status = 1;
    }
    rclcpp::shutdown();
    return status;
}
