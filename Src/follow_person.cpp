#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/float32.hpp>

namespace {

constexpr double kDefaultDistanceM = 3.5;
constexpr double kMinDistanceM = 2.0;
constexpr double kMaxDistanceM = 6.0;
constexpr double kMaxForwardSpeedMps = 0.6;
constexpr double kMaxStrafeSpeedMps = 0.35;
constexpr double kMaxYawRateDegS = 35.0;
constexpr auto kTargetLifetime = std::chrono::milliseconds(800);
constexpr auto kLoopPeriod = std::chrono::milliseconds(100);

[[nodiscard]] double clamp(double value, double min_value, double max_value) {
    return std::min(max_value, std::max(min_value, value));
}

[[nodiscard]] bool isFinite(const double value) {
    return std::isfinite(value);
}

}  // namespace

class FollowPersonNode : public rclcpp::Node {
public:
    FollowPersonNode()
        : Node("follow_person"),
          last_detection_time_(std::chrono::steady_clock::time_point::min()) {
        declare_parameter("position_topic", "/yolo_person_detector/person/position");
        declare_parameter("distance_topic", "/yolo_person_detector/person/distance");
        declare_parameter("camera_info_topic", "/camera/front/camera_info");
        declare_parameter("safe_distance_m", kDefaultDistanceM);
        declare_parameter("min_distance_m", kMinDistanceM);
        declare_parameter("max_distance_m", kMaxDistanceM);
        declare_parameter("max_forward_m_s", kMaxForwardSpeedMps);
        declare_parameter("max_strafe_m_s", kMaxStrafeSpeedMps);
        declare_parameter("max_yaw_rate_deg_s", kMaxYawRateDegS);
        declare_parameter("target_timeout_ms", static_cast<int>(kTargetLifetime.count()));

        position_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
            get_parameter("position_topic").as_string(), 10,
            [this](geometry_msgs::msg::PointStamped::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(state_mutex_);
                latest_position_ = *msg;
                last_detection_time_ = std::chrono::steady_clock::now();
                have_position_ = true;
            });

        distance_sub_ = create_subscription<std_msgs::msg::Float32>(
            get_parameter("distance_topic").as_string(), 10,
            [this](std_msgs::msg::Float32::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(state_mutex_);
                latest_distance_m_ = msg->data;
                have_distance_ = std::isfinite(msg->data) && msg->data > 0.0;
            });

        camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            get_parameter("camera_info_topic").as_string(), 10,
            [this](sensor_msgs::msg::CameraInfo::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(state_mutex_);
                image_center_x_ = static_cast<double>(msg->k[2]);
                image_width_ = static_cast<double>(msg->width);
                focal_length_x_ = static_cast<double>(msg->k[0]);
                have_camera_info_ = isFinite(image_center_x_) && isFinite(focal_length_x_) &&
                    focal_length_x_ > 0.0 && image_width_ > 0.0;
            });

        connectToVehicle();
        control_timer_ = create_wall_timer(kLoopPeriod, std::bind(&FollowPersonNode::controlLoop, this));

        RCLCPP_INFO(get_logger(), "Follow-person controller ready. Waiting for a target and armed vehicle.");
    }

private:
    void connectToVehicle() {
        mavsdk_ = std::make_unique<mavsdk::Mavsdk>(
            mavsdk::Mavsdk::Configuration{mavsdk::ComponentType::GroundStation});
        mavsdk_->set_timeout_s(3.0);

        const auto connection_result = mavsdk_->add_any_connection("udpin://0.0.0.0:14540");
        if (connection_result != mavsdk::ConnectionResult::Success) {
            throw std::runtime_error(
                "MAVSDK connection failed with code " + std::to_string(static_cast<int>(connection_result)));
        }

        std::shared_ptr<mavsdk::System> system;
        for (int attempt = 0; attempt < 30 && rclcpp::ok() && !system; ++attempt) {
            for (const auto& candidate : mavsdk_->systems()) {
                if (candidate->is_connected()) {
                    system = candidate;
                    break;
                }
            }
            if (!system) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }
        if (!system) {
            throw std::runtime_error("No PX4 vehicle found on MAVSDK UDP 14540");
        }

        action_ = std::make_unique<mavsdk::Action>(system);
        offboard_ = std::make_unique<mavsdk::Offboard>(system);
        telemetry_ = std::make_unique<mavsdk::Telemetry>(system);
    }

    void armAndTakeoffIfNeeded() {
        if (!telemetry_ || !action_) {
            return;
        }

        if (!telemetry_->armed()) {
            const auto arm_result = action_->arm();
            if (arm_result != mavsdk::Action::Result::Success) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "Arm command failed: %s",
                    std::to_string(static_cast<int>(arm_result)).c_str());
                return;
            }
            RCLCPP_INFO(get_logger(), "Vehicle armed.");
        }

        if (!telemetry_->in_air()) {
            const auto takeoff_result = action_->takeoff();
            if (takeoff_result != mavsdk::Action::Result::Success) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "Takeoff command failed: %s",
                    std::to_string(static_cast<int>(takeoff_result)).c_str());
                return;
            }
            RCLCPP_INFO(get_logger(), "Vehicle taking off.");
        }
    }

    void controlLoop() {
        if (!telemetry_ || !offboard_ || !action_) {
            return;
        }

        if (!telemetry_->armed() || !telemetry_->in_air()) {
            armAndTakeoffIfNeeded();
            return;
        }

        if (!offboard_started_) {
            const auto hold_result = offboard_->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});
            if (hold_result != mavsdk::Offboard::Result::Success) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "Prepare offboard setpoint failed: %s",
                    std::to_string(static_cast<int>(hold_result)).c_str());
                return;
            }
            const auto start_result = offboard_->start();
            if (start_result != mavsdk::Offboard::Result::Success) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "Offboard start failed: %s",
                    std::to_string(static_cast<int>(start_result)).c_str());
                return;
            }
            offboard_started_ = true;
            RCLCPP_INFO(get_logger(), "Offboard mode active; following target.");
        }

        geometry_msgs::msg::PointStamped position;
        double distance_m = 0.0;
        double center_x = 0.0;
        double focal_length = 0.0;
        double image_width = 0.0;
        bool have_position = false;
        bool have_distance = false;
        bool have_camera = false;

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            have_position = have_position_;
            if (have_position) {
                position = latest_position_;
            }
            have_distance = have_distance_;
            if (have_distance) {
                distance_m = latest_distance_m_;
            }
            have_camera = have_camera_info_;
            if (have_camera) {
                center_x = image_center_x_;
                focal_length = focal_length_x_;
                image_width = image_width_;
            }
        }

        const auto timeout_ms = get_parameter("target_timeout_ms").as_int();
        const auto target_age = std::chrono::steady_clock::now() - last_detection_time_;
        const bool target_is_fresh = have_position && target_age <= std::chrono::milliseconds(timeout_ms);

        double forward_m_s = 0.0;
        double right_m_s = 0.0;
        double yawspeed_deg_s = 0.0;

        if (target_is_fresh && have_camera) {
            const double offset_px = position.point.x - center_x;
            const double normalized = clamp(offset_px / (image_width * 0.5), -1.0, 1.0);
            const double safe_distance = get_parameter("safe_distance_m").as_double();
            const double min_distance = get_parameter("min_distance_m").as_double();
            const double max_distance = get_parameter("max_distance_m").as_double();
            const double max_forward = get_parameter("max_forward_m_s").as_double();
            const double max_strafe = get_parameter("max_strafe_m_s").as_double();
            const double max_yaw = get_parameter("max_yaw_rate_deg_s").as_double();

            if (have_distance) {
                distance_m = clamp(distance_m, min_distance, max_distance);
                const double distance_error = distance_m - safe_distance;
                forward_m_s = clamp(distance_error * 0.35, -max_forward, max_forward);
            }

            const double lateral_error = clamp(normalized * 0.7, -1.0, 1.0);
            right_m_s = clamp(lateral_error * max_strafe, -max_strafe, max_strafe);
            yawspeed_deg_s = clamp(normalized * max_yaw, -max_yaw, max_yaw);

            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "target offset=%.1f px, distance=%.2f m, forward=%.2f m/s, yaw=%.1f deg/s",
                offset_px, distance_m, forward_m_s, yawspeed_deg_s);
        } else {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No fresh target; hovering.");
        }

        const auto result = offboard_->set_velocity_body({
            static_cast<float>(forward_m_s),
            static_cast<float>(right_m_s),
            0.0f,
            static_cast<float>(yawspeed_deg_s)});

        if (result != mavsdk::Offboard::Result::Success) {
            RCLCPP_ERROR_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Follow command failed: %s",
                std::to_string(static_cast<int>(result)).c_str());
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr position_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr distance_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    std::unique_ptr<mavsdk::Mavsdk> mavsdk_;
    std::unique_ptr<mavsdk::Action> action_;
    std::unique_ptr<mavsdk::Offboard> offboard_;
    std::unique_ptr<mavsdk::Telemetry> telemetry_;

    std::mutex state_mutex_;
    geometry_msgs::msg::PointStamped latest_position_{};
    double latest_distance_m_{0.0};
    double image_center_x_{0.0};
    double focal_length_x_{0.0};
    double image_width_{0.0};
    bool have_position_{false};
    bool have_distance_{false};
    bool have_camera_info_{false};
    bool offboard_started_{false};
    std::chrono::steady_clock::time_point last_detection_time_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<FollowPersonNode>();
        rclcpp::spin(node);
    } catch (const std::exception& error) {
        RCLCPP_ERROR(rclcpp::get_logger("follow_person"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
