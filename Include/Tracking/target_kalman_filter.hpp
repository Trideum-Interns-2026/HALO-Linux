#ifndef TARGET_KALMAN_FILTER_HPP
#define TARGET_KALMAN_FILTER_HPP

#include <cmath>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include <Eigen/Dense>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/float32.hpp>

#include "Groundstation/mavlink_interface.hpp"

class TargetKalmanFilterNode : public rclcpp::Node {
public:
    TargetKalmanFilterNode()
        : Node("target_kalman_filter"),
          state_(Eigen::Vector4d::Zero()),
          covariance_(Eigen::Matrix4d::Identity() * 100.0),
          process_noise_(Eigen::Matrix4d::Identity() * 0.05),
          measurement_noise_(Eigen::Matrix2d::Identity() * 0.09),
          last_measurement_time_(this->now()) {
        declare_parameter("position_topic", "/red_sphere_detector/red_sphere/position");
        declare_parameter("distance_topic", "/red_sphere_detector/red_sphere/distance");
        declare_parameter("camera_info_topic", "/camera/front/camera_info");
        declare_parameter("mavlink_endpoint", "127.0.0.1");
        declare_parameter("mavlink_port", 14550);
        declare_parameter("mavsdk_connection_url", "udp://:14540");
        declare_parameter("roi_period_ms", 200);
        declare_parameter("vehicle_latitude", 0.0);
        declare_parameter("vehicle_longitude", 0.0);
        declare_parameter("vehicle_altitude_m", 0.0);
        declare_parameter("vehicle_heading_deg", 0.0);
        declare_parameter("camera_yaw_offset_deg", 0.0);
        declare_parameter("camera_pitch_offset_deg", 0.0);
        declare_parameter("target_system_id", 1);
        declare_parameter("target_component_id", 1);

        vehicle_latitude_ = get_parameter("vehicle_latitude").as_double();
        vehicle_longitude_ = get_parameter("vehicle_longitude").as_double();
        vehicle_altitude_m_ = get_parameter("vehicle_altitude_m").as_double();
        vehicle_heading_deg_ = get_parameter("vehicle_heading_deg").as_double();

        const auto position_topic = get_parameter("position_topic").as_string();
        const auto distance_topic = get_parameter("distance_topic").as_string();
        const auto camera_info_topic = get_parameter("camera_info_topic").as_string();

        sender_ = std::make_unique<MavlinkSender>(
            get_parameter("mavlink_endpoint").as_string(),
            get_parameter("mavlink_port").as_int());

        position_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
            position_topic, 10,
            std::bind(&TargetKalmanFilterNode::positionCallback, this,
                      std::placeholders::_1));
        distance_sub_ = create_subscription<std_msgs::msg::Float32>(
            distance_topic, 10,
            std::bind(&TargetKalmanFilterNode::distanceCallback, this,
                      std::placeholders::_1));
        camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic, 10,
            std::bind(&TargetKalmanFilterNode::cameraInfoCallback, this,
                      std::placeholders::_1));

        last_target_time_ = now();
        roi_timer_ = create_wall_timer(
            std::chrono::milliseconds(get_parameter("roi_period_ms").as_int()),
            std::bind(&TargetKalmanFilterNode::publishRoi, this));

        mavsdk_.add_any_connection(get_parameter("mavsdk_connection_url").as_string());
        telemetry_timer_ = create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&TargetKalmanFilterNode::connectTelemetry, this));

        RCLCPP_INFO(get_logger(), "Tracking target from '%s' and publishing MAVLink ROI updates",
                    position_topic.c_str());
    }

private:
    void positionCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_pixel_ = *msg;
        have_pixel_ = true;
        processMeasurementLocked();
    }

    void distanceCallback(const std_msgs::msg::Float32::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_distance_m_ = msg->data;
        have_distance_ = std::isfinite(latest_distance_m_) && latest_distance_m_ > 0.0;
        processMeasurementLocked();
    }

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        fx_ = msg->k[0];
        fy_ = msg->k[4];
        cx_ = msg->k[2];
        cy_ = msg->k[5];
        have_camera_info_ = fx_ > 0.0 && fy_ > 0.0;
    }

    void processMeasurementLocked() {
        if (!have_pixel_ || !have_distance_ || !have_camera_info_ ||
            !have_vehicle_telemetry_) {
            return;
        }

        const auto stamp = rclcpp::Time(latest_pixel_.header.stamp);
        const double dt = (stamp - last_measurement_time_).seconds();
        if (initialized_ && dt <= 0.0) {
            return;
        }

        const double camera_yaw = degToRad(get_parameter("camera_yaw_offset_deg").as_double());
        const double camera_pitch = degToRad(get_parameter("camera_pitch_offset_deg").as_double());
        const double ray_right = (latest_pixel_.point.x - cx_) / fx_;
        const double ray_down = (latest_pixel_.point.y - cy_) / fy_;
        const double ray_forward = 1.0;
        const double ray_length = std::sqrt(
            ray_right * ray_right + ray_down * ray_down + ray_forward * ray_forward);
        const double right = ray_right / ray_length;
        const double down = ray_down / ray_length;
        const double forward = ray_forward / ray_length;
        const double horizontal_forward = forward * std::cos(camera_pitch) - down * std::sin(camera_pitch);
        const double target_down = latest_distance_m_ *
            (forward * std::sin(camera_pitch) + down * std::cos(camera_pitch));
        const double heading = degToRad(vehicle_heading_deg_) + camera_yaw;
        const double north_camera = latest_distance_m_ * horizontal_forward;
        const double east_camera = latest_distance_m_ * right;
        const double north = north_camera * std::cos(heading) - east_camera * std::sin(heading);
        const double east = north_camera * std::sin(heading) + east_camera * std::cos(heading);
        if (!filter_reference_set_) {
            filter_reference_latitude_ = vehicle_latitude_;
            filter_reference_longitude_ = vehicle_longitude_;
            filter_reference_set_ = true;
        }

        const double vehicle_north =
            (vehicle_latitude_ - filter_reference_latitude_) * 111111.0;
        const double vehicle_east =
            (vehicle_longitude_ - filter_reference_longitude_) *
            (111111.0 * std::cos(degToRad(filter_reference_latitude_)));
        const double target_north = vehicle_north + north;
        const double target_east = vehicle_east + east;

        if (initialized_) {
            predict(dt);
        } else {
            state_.setZero();
            state_(0) = target_north;
            state_(1) = target_east;
            covariance_ = Eigen::Matrix4d::Identity();
            initialized_ = true;
        }

        update(Eigen::Vector2d(target_north, target_east));
        target_down_m_ = target_down;
        last_measurement_time_ = stamp;
        last_target_time_ = now();
        have_pixel_ = false;
        have_distance_ = false;
    }

    void predict(double dt) {
        Eigen::Matrix4d transition = Eigen::Matrix4d::Identity();
        transition(0, 2) = dt;
        transition(1, 3) = dt;
        state_ = transition * state_;
        covariance_ = transition * covariance_ * transition.transpose() + process_noise_;
    }

    void update(const Eigen::Vector2d &measurement) {
        Eigen::Matrix<double, 2, 4> measurement_matrix = Eigen::Matrix<double, 2, 4>::Zero();
        measurement_matrix(0, 0) = 1.0;
        measurement_matrix(1, 1) = 1.0;
        const Eigen::Vector2d residual = measurement - measurement_matrix * state_;
        const Eigen::Matrix2d innovation =
            measurement_matrix * covariance_ * measurement_matrix.transpose() + measurement_noise_;
        const Eigen::Matrix<double, 4, 2> gain =
            covariance_ * measurement_matrix.transpose() * innovation.inverse();
        state_ += gain * residual;
        covariance_ = (Eigen::Matrix4d::Identity() - gain * measurement_matrix) * covariance_;
    }

    void publishRoi() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!have_vehicle_telemetry_ || !initialized_ ||
            (now() - last_target_time_).seconds() > 2.0) {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Waiting for vehicle telemetry and a fresh target detection "
                "(vehicle %.7f, %.7f, %.1fm)",
                vehicle_latitude_, vehicle_longitude_, vehicle_altitude_m_);
            return;
        }

        const double latitude = filter_reference_latitude_ + state_(0) / 111111.0;
        const double longitude = filter_reference_longitude_ +
            state_(1) / (111111.0 * std::cos(degToRad(filter_reference_latitude_)));
        const double altitude = vehicle_altitude_m_ - target_down_m_;

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "DRONE lat=%.7f lon=%.7f alt=%.2fm | TARGET lat=%.7f lon=%.7f alt=%.2fm | distance=%.2fm",
            vehicle_latitude_, vehicle_longitude_, vehicle_altitude_m_,
            latitude, longitude, altitude, latest_distance_m_);

        sender_->sendRoiLocation(
            latitude,
            longitude,
            static_cast<float>(altitude),
            static_cast<uint8_t>(get_parameter("target_system_id").as_int()),
            static_cast<uint8_t>(get_parameter("target_component_id").as_int()));
    }

    void connectTelemetry() {
        if (telemetry_) {
            return;
        }
        for (const auto &system : mavsdk_.systems()) {
            if (!system->is_connected()) {
                continue;
            }
            telemetry_ = std::make_unique<mavsdk::Telemetry>(system);
            telemetry_->subscribe_position([this](mavsdk::Telemetry::Position position) {
                std::lock_guard<std::mutex> lock(data_mutex_);
                vehicle_latitude_ = position.latitude_deg;
                vehicle_longitude_ = position.longitude_deg;
                vehicle_altitude_m_ = position.absolute_altitude_m;
                have_vehicle_telemetry_ = true;
            });
            telemetry_->subscribe_heading([this](mavsdk::Telemetry::Heading heading) {
                std::lock_guard<std::mutex> lock(data_mutex_);
                vehicle_heading_deg_ = heading.heading_deg;
            });
            RCLCPP_INFO(get_logger(), "Connected to vehicle telemetry");
            return;
        }
    }

    static double degToRad(double degrees) {
        return degrees * 3.14159265358979323846 / 180.0;
    }

    std::unique_ptr<MavlinkSender> sender_;
    mavsdk::Mavsdk mavsdk_{mavsdk::Mavsdk::Configuration{mavsdk::ComponentType::GroundStation}};
    std::unique_ptr<mavsdk::Telemetry> telemetry_;

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr position_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr distance_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::TimerBase::SharedPtr roi_timer_;
    rclcpp::TimerBase::SharedPtr telemetry_timer_;

    std::mutex data_mutex_;
    geometry_msgs::msg::PointStamped latest_pixel_;
    double latest_distance_m_{0.0};
    double fx_{0.0};
    double fy_{0.0};
    double cx_{0.0};
    double cy_{0.0};
    double vehicle_latitude_{0.0};
    double vehicle_longitude_{0.0};
    double vehicle_altitude_m_{0.0};
    double vehicle_heading_deg_{0.0};
    double target_down_m_{0.0};
    double filter_reference_latitude_{0.0};
    double filter_reference_longitude_{0.0};
    rclcpp::Time last_measurement_time_;
    rclcpp::Time last_target_time_;
    Eigen::Vector4d state_;
    Eigen::Matrix4d covariance_;
    Eigen::Matrix4d process_noise_;
    Eigen::Matrix2d measurement_noise_;
    bool have_pixel_{false};
    bool have_distance_{false};
    bool have_camera_info_{false};
    bool have_vehicle_telemetry_{false};
    bool filter_reference_set_{false};
    bool initialized_{false};
};

#endif