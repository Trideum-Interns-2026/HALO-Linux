#ifndef GEOLOCATION_BUILDING_RISK_NODE_HPP
#define GEOLOCATION_BUILDING_RISK_NODE_HPP

#include <array>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geolocation_msgs/msg/building_risk.hpp>
#include <geolocation_msgs/msg/geolocation_state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "geolocation/building_occlusion.hpp"

class BuildingRiskNode : public rclcpp::Node {
public:
    explicit BuildingRiskNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("building_risk", options) {
        const std::string default_buildings_file =
            ament_index_cpp::get_package_share_directory("geolocation") +
            "/config/buildings_huntsville.csv";
        declare_parameter("input_topic", "/geolocation/position");
        declare_parameter("output_topic", "/geolocation/building_risk");
        declare_parameter("buildings_file", default_buildings_file);
        declare_parameter("max_range_m", 200.0);

        input_topic_ = get_parameter("input_topic").as_string();
        output_topic_ = get_parameter("output_topic").as_string();
        buildings_file_ = get_parameter("buildings_file").as_string();
        max_range_m_ = get_parameter("max_range_m").as_double();
        if (!std::isfinite(max_range_m_) || max_range_m_ < 0.0) {
            throw std::invalid_argument("max_range_m must be finite and non-negative");
        }

        const auto loaded = geolocation::loadBuildingFootprints(buildings_file_);
        buildings_ = loaded.footprints;
        if (!loaded.file_opened || !loaded.header_valid || loaded.skipped_rows > 0) {
            RCLCPP_WARN(
                get_logger(),
                "building data issue: file=%s opened=%s header_valid=%s skipped_rows=%zu",
                buildings_file_.c_str(), loaded.file_opened ? "true" : "false",
                loaded.header_valid ? "true" : "false", loaded.skipped_rows);
        }

        position_sub_ = create_subscription<geolocation_msgs::msg::GeolocationState>(
            input_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
            std::bind(&BuildingRiskNode::onPosition, this, std::placeholders::_1));
        risk_pub_ = create_publisher<geolocation_msgs::msg::BuildingRisk>(
            output_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

        RCLCPP_INFO(
            get_logger(),
            "building risk ready: input_topic=%s output_topic=%s buildings_file=%s "
            "loaded_buildings=%zu max_range_m=%.1f",
            input_topic_.c_str(), output_topic_.c_str(), buildings_file_.c_str(),
            buildings_.size(), max_range_m_);
    }

    void onPosition(const geolocation_msgs::msg::GeolocationState::SharedPtr message) {
        if (!message) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000, "Skipping null geolocation position");
            return;
        }
        if (buildings_.empty()) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "No building footprints available; skipping building risk output");
            return;
        }
        const geolocation::GeodeticPosition position{
            message->latitude_deg, message->longitude_deg, message->altitude_m};
        const std::array<double, 3> errors{
            message->error_x_m, message->error_y_m, message->error_z_m};
        try {
            const auto estimate = geolocation::estimateBuildingRisk(
                position, errors, buildings_, max_range_m_);
            geolocation_msgs::msg::BuildingRisk output;
            output.header = message->header;
            output.probability = estimate.probability;
            output.building_bearing_deg = estimate.building_bearing_deg;
            output.building_distance_m = estimate.building_distance_m;
            output.building_height_m = estimate.building_height_m;
            risk_pub_->publish(output);
        } catch (const std::out_of_range&) {
            return;
        } catch (const std::invalid_argument& error) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "Skipping invalid geolocation position for building risk: %s", error.what());
        }
    }

private:
    std::string input_topic_;
    std::string output_topic_;
    std::string buildings_file_;
    double max_range_m_{};
    std::vector<geolocation::BuildingFootprint> buildings_;
    rclcpp::Subscription<geolocation_msgs::msg::GeolocationState>::SharedPtr position_sub_;
    rclcpp::Publisher<geolocation_msgs::msg::BuildingRisk>::SharedPtr risk_pub_;
};

#endif
