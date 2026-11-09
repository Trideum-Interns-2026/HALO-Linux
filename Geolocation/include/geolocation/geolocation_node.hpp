#ifndef GEOLOCATION_GEOLOCATION_NODE_HPP
#define GEOLOCATION_GEOLOCATION_NODE_HPP

#include <array>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <geolocation_msgs/msg/geolocation_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>

#include "geolocation/position_estimator.hpp"

class GeolocationNode : public rclcpp::Node {
public:
    GeolocationNode() : Node("geolocation") {
        declare_parameter("gps_topic", "/gps/fix");
        declare_parameter("output_topic", "/geolocation/position");
        declare_parameter("horizontal_stddev_m", 1.0);
        declare_parameter("vertical_stddev_m", 1.5);
        declare_parameter("process_noise_variance_m2_per_s", 0.25);

        gps_topic_ = get_parameter("gps_topic").as_string();
        output_topic_ = get_parameter("output_topic").as_string();
        const double horizontal_stddev = get_parameter("horizontal_stddev_m").as_double();
        const double vertical_stddev = get_parameter("vertical_stddev_m").as_double();
        const double process_noise =
            get_parameter("process_noise_variance_m2_per_s").as_double();
        estimator_ = std::make_unique<geolocation::PositionEstimator>(
            horizontal_stddev, vertical_stddev, process_noise);

        gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
            gps_topic_, rclcpp::SensorDataQoS(),
            std::bind(&GeolocationNode::onGps, this, std::placeholders::_1));
        output_pub_ = create_publisher<geolocation_msgs::msg::GeolocationState>(
            output_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

        RCLCPP_INFO(
            get_logger(),
            "geolocation ready: gps_topic=%s output_topic=%s horizontal_stddev_m=%.3f "
            "vertical_stddev_m=%.3f process_noise_variance_m2_per_s=%.3f",
            gps_topic_.c_str(), output_topic_.c_str(), horizontal_stddev,
            vertical_stddev, process_noise);
    }

    void onGps(const sensor_msgs::msg::NavSatFix::SharedPtr message) {
        if (!message || message->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX ||
            !std::isfinite(message->latitude) || !std::isfinite(message->longitude) ||
            !std::isfinite(message->altitude)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000, "Skipping invalid or unavailable GPS fix");
            return;
        }

        const auto measurement_time = message->header.stamp.sec == 0 &&
                message->header.stamp.nanosec == 0 ?
            now() : rclcpp::Time(message->header.stamp, get_clock()->get_clock_type());
        std::lock_guard<std::mutex> lock(mutex_);
        const double elapsed_seconds = last_measurement_time_.has_value() ?
            (measurement_time - *last_measurement_time_).seconds() : 0.0;
        if (elapsed_seconds < 0.0) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000, "Skipping out-of-order GPS fix");
            return;
        }

        const geolocation::GeodeticPosition position{
            message->latitude, message->longitude, message->altitude};
        try {
            const auto estimate = estimator_->update(
                position, measurementVariance(*message), elapsed_seconds);
            geolocation_msgs::msg::GeolocationState output;
            output.header = message->header;
            output.latitude_deg = estimate.position.latitude_deg;
            output.longitude_deg = estimate.position.longitude_deg;
            output.altitude_m = estimate.position.altitude_m;
            output.error_x_m = estimate.error_ecef_3sigma_m[0];
            output.error_y_m = estimate.error_ecef_3sigma_m[1];
            output.error_z_m = estimate.error_ecef_3sigma_m[2];
            output_pub_->publish(output);
        } catch (const std::invalid_argument& error) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000, "Skipping invalid GPS fix: %s", error.what());
            return;
        }

        last_measurement_time_ = measurement_time;
    }

private:
    static std::array<double, 3> measurementVariance(
        const sensor_msgs::msg::NavSatFix& message) {
        if (message.position_covariance_type ==
                sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
            return {};
        }
        return {
            message.position_covariance[0],
            message.position_covariance[4],
            message.position_covariance[8]
        };
    }

    std::mutex mutex_;
    std::string gps_topic_;
    std::string output_topic_;
    std::unique_ptr<geolocation::PositionEstimator> estimator_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Publisher<geolocation_msgs::msg::GeolocationState>::SharedPtr output_pub_;
    std::optional<rclcpp::Time> last_measurement_time_;
};

#endif
