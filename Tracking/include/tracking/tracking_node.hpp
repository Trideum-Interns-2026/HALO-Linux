#ifndef TRACKING_TRACKING_NODE_HPP
#define TRACKING_TRACKING_NODE_HPP

#include <array>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include <geolocation_msgs/msg/geolocation_state.hpp>
#include <perception_msgs/msg/detection.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tracking_msgs/msg/tracking_state.hpp>

#include "tracking/target_estimator.hpp"

class TrackingNode : public rclcpp::Node {
public:
    TrackingNode() : Node("tracking") {
        declare_parameter("detection_topic", "/perception/detection");
        declare_parameter("camera_info_topic", "/camera/front/camera_info");
        declare_parameter("geolocation_topic", "/geolocation/position");
        declare_parameter("vehicle_attitude_topic", "/fmu/out/vehicle_attitude");
        declare_parameter("output_topic", "/tracking/position");
        declare_parameter("human_height_m", 1.75);
        declare_parameter("human_height_stddev_m", 0.15);
        declare_parameter("pixel_stddev_px", 2.0);
        declare_parameter("minimum_confidence", 0.5);
        declare_parameter("maximum_input_age_s", 0.5);
        declare_parameter("camera_roll_deg", 0.0);
        declare_parameter("camera_pitch_deg", 0.0);
        declare_parameter("camera_yaw_deg", 0.0);
        declare_parameter("frame_edge_margin_px", tracking::kDefaultFrameEdgeMarginPx);

        detection_topic_ = get_parameter("detection_topic").as_string();
        camera_info_topic_ = get_parameter("camera_info_topic").as_string();
        geolocation_topic_ = get_parameter("geolocation_topic").as_string();
        vehicle_attitude_topic_ = get_parameter("vehicle_attitude_topic").as_string();
        output_topic_ = get_parameter("output_topic").as_string();
        human_height_m_ = get_parameter("human_height_m").as_double();
        human_height_stddev_m_ = get_parameter("human_height_stddev_m").as_double();
        pixel_stddev_px_ = get_parameter("pixel_stddev_px").as_double();
        minimum_confidence_ = get_parameter("minimum_confidence").as_double();
        maximum_input_age_s_ = get_parameter("maximum_input_age_s").as_double();
        camera_roll_rad_ = toRadians(get_parameter("camera_roll_deg").as_double());
        camera_pitch_rad_ = toRadians(get_parameter("camera_pitch_deg").as_double());
        camera_yaw_rad_ = toRadians(get_parameter("camera_yaw_deg").as_double());
        frame_edge_margin_px_ = tracking::sanitizeFrameEdgeMargin(
            get_parameter("frame_edge_margin_px").as_double());
        validateParameters();

        detection_sub_ = create_subscription<perception_msgs::msg::Detection>(
            detection_topic_, rclcpp::SensorDataQoS(),
            std::bind(&TrackingNode::onDetection, this, std::placeholders::_1));
        camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic_, rclcpp::SensorDataQoS(),
            std::bind(&TrackingNode::onCameraInfo, this, std::placeholders::_1));
        geolocation_sub_ = create_subscription<geolocation_msgs::msg::GeolocationState>(
            geolocation_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
            std::bind(&TrackingNode::onGeolocation, this, std::placeholders::_1));
        attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
            vehicle_attitude_topic_, rclcpp::SensorDataQoS(),
            std::bind(&TrackingNode::onAttitude, this, std::placeholders::_1));
        output_pub_ = create_publisher<tracking_msgs::msg::TrackingState>(
            output_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

        RCLCPP_INFO(
            get_logger(),
            "tracking ready: detection=%s camera_info=%s geolocation=%s attitude=%s "
            "output=%s human_height_m=%.3f human_height_stddev_m=%.3f "
            "pixel_stddev_px=%.3f minimum_confidence=%.3f maximum_input_age_s=%.3f "
            "camera_mount_rpy_deg=(%.3f,%.3f,%.3f) frame_edge_margin_px=%.3f",
            detection_topic_.c_str(), camera_info_topic_.c_str(),
            geolocation_topic_.c_str(), vehicle_attitude_topic_.c_str(),
            output_topic_.c_str(), human_height_m_, human_height_stddev_m_,
            pixel_stddev_px_, minimum_confidence_, maximum_input_age_s_,
            camera_roll_rad_ * 180.0 / tracking::kPi,
            camera_pitch_rad_ * 180.0 / tracking::kPi,
            camera_yaw_rad_ * 180.0 / tracking::kPi, frame_edge_margin_px_);
    }

    void onCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr message) {
        if (!message || !std::isfinite(message->k[0]) || !std::isfinite(message->k[4]) ||
            message->k[0] <= 0.0 || message->k[4] <= 0.0 ||
            !std::isfinite(message->k[2]) || !std::isfinite(message->k[5])) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000, "Skipping invalid camera calibration");
            return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        camera_ = tracking::CameraIntrinsics{
            message->k[0], message->k[4], message->k[2], message->k[5]};
        image_height_px_ = static_cast<double>(message->height);
        camera_ready_ = true;
    }

    void onGeolocation(
        const geolocation_msgs::msg::GeolocationState::SharedPtr message) {
        if (!message || !std::isfinite(message->latitude_deg) ||
            !std::isfinite(message->longitude_deg) || !std::isfinite(message->altitude_m) ||
            message->latitude_deg < -90.0 || message->latitude_deg > 90.0 ||
            message->longitude_deg < -180.0 || message->longitude_deg > 180.0 ||
            !std::isfinite(message->error_x_m) || message->error_x_m < 0.0 ||
            !std::isfinite(message->error_y_m) || message->error_y_m < 0.0 ||
            !std::isfinite(message->error_z_m) || message->error_z_m < 0.0) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000, "Skipping invalid drone position estimate");
            return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        geolocation_ = *message;
        geolocation_received_at_ = now();
        geolocation_ready_ = true;
    }

    void onAttitude(const px4_msgs::msg::VehicleAttitude::SharedPtr message) {
        if (!message) {
            return;
        }
        const tracking::Quaternion attitude{
            message->q[0], message->q[1], message->q[2], message->q[3]};
        const double norm = std::sqrt(
            attitude.w * attitude.w + attitude.x * attitude.x +
            attitude.y * attitude.y + attitude.z * attitude.z);
        if (!std::isfinite(norm) || norm <= 0.0) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000, "Skipping invalid PX4 attitude quaternion");
            return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        attitude_ = attitude;
        attitude_received_at_ = now();
        attitude_ready_ = true;
    }

    void onDetection(const perception_msgs::msg::Detection::SharedPtr message) {
        if (!message) {
            return;
        }

        tracking::CameraIntrinsics camera;
        double image_height_px = 0.0;
        geolocation_msgs::msg::GeolocationState geolocation;
        tracking::Quaternion attitude;
        rclcpp::Time geolocation_received_at(0, 0, get_clock()->get_clock_type());
        rclcpp::Time attitude_received_at(0, 0, get_clock()->get_clock_type());
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!camera_ready_ || !geolocation_ready_ || !attitude_ready_) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 5000,
                    "Waiting for camera calibration, drone position and PX4 attitude");
                return;
            }
            camera = camera_;
            image_height_px = image_height_px_;
            geolocation = geolocation_;
            attitude = attitude_;
            geolocation_received_at = geolocation_received_at_;
            attitude_received_at = attitude_received_at_;
        }

        const auto current_time = now();
        const rclcpp::Time detection_time(
            message->header.stamp, get_clock()->get_clock_type());
        const bool detection_stamp_set =
            message->header.stamp.sec != 0 || message->header.stamp.nanosec != 0;
        const auto capture_time = detection_stamp_set ? detection_time : current_time;
        const bool geolocation_stamp_set =
            geolocation.header.stamp.sec != 0 || geolocation.header.stamp.nanosec != 0;
        const auto position_time = geolocation_stamp_set ?
            rclcpp::Time(geolocation.header.stamp, get_clock()->get_clock_type()) :
            geolocation_received_at;
        if (isStale(current_time, capture_time) ||
            isStale(current_time, geolocation_received_at) ||
            isStale(current_time, attitude_received_at) ||
            isStale(capture_time, position_time)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000, "Skipping detection with stale pose inputs");
            return;
        }

        // A box cut by the top or bottom edge is too short: ranging would place the person too
        // far away. Publish nothing rather than a false position.
        if (tracking::boxTouchesVerticalFrameEdge(
                message->y_min, message->y_max, image_height_px, frame_edge_margin_px_)) {
            RCLCPP_DEBUG_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "Skipping detection clipped by the top or bottom image edge");
            return;
        }

        const tracking::DetectionBox box{
            message->confidence, message->x_min, message->y_min,
            message->x_max, message->y_max};
        const tracking::GeodeticPosition drone_position{
            geolocation.latitude_deg, geolocation.longitude_deg, geolocation.altitude_m};
        const std::array<double, 3> drone_error{
            geolocation.error_x_m, geolocation.error_y_m, geolocation.error_z_m};
        try {
            const auto estimate = tracking::estimateTarget(
                box, camera, drone_position, drone_error, attitude, human_height_m_,
                human_height_stddev_m_, pixel_stddev_px_, camera_roll_rad_,
                camera_pitch_rad_, camera_yaw_rad_, minimum_confidence_);
            tracking_msgs::msg::TrackingState output;
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
                get_logger(), *get_clock(), 5000,
                "Skipping invalid person detection: %s", error.what());
        }
    }

private:
    static double toRadians(double degrees) {
        return degrees * tracking::kPi / 180.0;
    }

    void validateParameters() const {
        if (!std::isfinite(human_height_m_) || human_height_m_ <= 0.0 ||
            !std::isfinite(human_height_stddev_m_) || human_height_stddev_m_ <= 0.0 ||
            !std::isfinite(pixel_stddev_px_) || pixel_stddev_px_ < 0.0 ||
            !std::isfinite(minimum_confidence_) || minimum_confidence_ < 0.0 ||
            minimum_confidence_ > 1.0 || !std::isfinite(maximum_input_age_s_) ||
            maximum_input_age_s_ <= 0.0 || !std::isfinite(camera_roll_rad_) ||
            !std::isfinite(camera_pitch_rad_) || !std::isfinite(camera_yaw_rad_)) {
            throw std::invalid_argument("tracking parameters are outside valid ranges");
        }
    }

    bool isStale(const rclcpp::Time& current, const rclcpp::Time& input) const {
        return std::abs((current - input).seconds()) > maximum_input_age_s_;
    }

    std::mutex mutex_;
    std::string detection_topic_;
    std::string camera_info_topic_;
    std::string geolocation_topic_;
    std::string vehicle_attitude_topic_;
    std::string output_topic_;
    double human_height_m_{};
    double human_height_stddev_m_{};
    double pixel_stddev_px_{};
    double minimum_confidence_{};
    double maximum_input_age_s_{};
    double camera_roll_rad_{};
    double camera_pitch_rad_{};
    double camera_yaw_rad_{};
    double frame_edge_margin_px_{tracking::kDefaultFrameEdgeMarginPx};
    double image_height_px_{0.0};  // from camera_info.height; 0 = unknown
    tracking::CameraIntrinsics camera_{};
    geolocation_msgs::msg::GeolocationState geolocation_{};
    tracking::Quaternion attitude_{};
    rclcpp::Time geolocation_received_at_{0, 0, RCL_ROS_TIME};
    rclcpp::Time attitude_received_at_{0, 0, RCL_ROS_TIME};
    bool camera_ready_{false};
    bool geolocation_ready_{false};
    bool attitude_ready_{false};
    rclcpp::Subscription<perception_msgs::msg::Detection>::SharedPtr detection_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<geolocation_msgs::msg::GeolocationState>::SharedPtr geolocation_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
    rclcpp::Publisher<tracking_msgs::msg::TrackingState>::SharedPtr output_pub_;
};

#endif
