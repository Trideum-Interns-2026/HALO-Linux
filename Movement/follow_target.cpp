#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/float32.hpp>

using namespace std::chrono_literals;

class FollowTargetNode : public rclcpp::Node {
public:
    FollowTargetNode()
        : Node("follow_target"),
          mavsdk_(mavsdk::Mavsdk::Configuration{mavsdk::ComponentType::GroundStation}) {
        declare_parameter("position_topic", "/red_sphere_detector/red_sphere/position");
        declare_parameter("distance_topic", "/red_sphere_detector/red_sphere/distance");
        declare_parameter("camera_info_topic", "/camera/front/camera_info");
        declare_parameter("mavsdk_connection_url", "udp://:14540");
        declare_parameter("follow_distance_m", 3.0);
        declare_parameter("max_forward_speed_mps", 0.8);
        declare_parameter("max_lateral_speed_mps", 0.5);
        declare_parameter("forward_gain", 0.5);
        declare_parameter("lateral_gain_mps", 0.6);
        declare_parameter("max_detection_distance_m", 30.0);
        declare_parameter("target_timeout_ms", 1500);
        declare_parameter("control_period_ms", 100);

        follow_distance_m_ = get_parameter("follow_distance_m").as_double();
        max_forward_speed_mps_ = get_parameter("max_forward_speed_mps").as_double();
        max_lateral_speed_mps_ = get_parameter("max_lateral_speed_mps").as_double();
        forward_gain_ = get_parameter("forward_gain").as_double();
        lateral_gain_mps_ = get_parameter("lateral_gain_mps").as_double();
        max_detection_distance_m_ = get_parameter("max_detection_distance_m").as_double();
        target_timeout_ = std::chrono::milliseconds(
            get_parameter("target_timeout_ms").as_int());

        const auto position_topic = get_parameter("position_topic").as_string();
        const auto distance_topic = get_parameter("distance_topic").as_string();
        const auto camera_info_topic = get_parameter("camera_info_topic").as_string();

        position_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
            position_topic, 10,
            [this](geometry_msgs::msg::PointStamped::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(data_mutex_);
                pixel_x_ = msg->point.x;
                last_position_time_ = std::chrono::steady_clock::now();
                have_position_ = std::isfinite(pixel_x_);
            });
        distance_sub_ = create_subscription<std_msgs::msg::Float32>(
            distance_topic, 10,
            [this](std_msgs::msg::Float32::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(data_mutex_);
                distance_m_ = msg->data;
                last_distance_time_ = std::chrono::steady_clock::now();
                have_distance_ = std::isfinite(distance_m_) && distance_m_ > 0.0;
            });
        camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic, 10,
            [this](sensor_msgs::msg::CameraInfo::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(data_mutex_);
                image_center_x_ = msg->k[2];
                have_camera_info_ = std::isfinite(image_center_x_) && image_center_x_ > 0.0;
            });

        const auto connection_result = mavsdk_.add_any_connection(
            get_parameter("mavsdk_connection_url").as_string());
        if (connection_result != mavsdk::ConnectionResult::Success) {
            RCLCPP_ERROR(get_logger(), "MAVSDK connection setup failed: %s",
                         mavsdk::to_string(connection_result).c_str());
        }

        connection_timer_ = create_wall_timer(
            500ms, std::bind(&FollowTargetNode::connectVehicle, this));
        control_timer_ = create_wall_timer(
            std::chrono::milliseconds(get_parameter("control_period_ms").as_int()),
            std::bind(&FollowTargetNode::controlStep, this));

        RCLCPP_WARN(get_logger(),
                    "Follow controller ready. It will not arm or take off; "
                    "use only in a safe test environment.");
    }

    ~FollowTargetNode() override {
        stopOffboard();
    }

private:
    void connectVehicle() {
        if (telemetry_) {
            return;
        }

        for (const auto &system : mavsdk_.systems()) {
            if (!system->is_connected()) {
                continue;
            }

            telemetry_ = std::make_unique<mavsdk::Telemetry>(system);
            offboard_ = std::make_unique<mavsdk::Offboard>(system);
            telemetry_->subscribe_armed([this](bool armed) {
                std::lock_guard<std::mutex> lock(data_mutex_);
                armed_ = armed;
            });
            telemetry_->subscribe_in_air([this](bool in_air) {
                std::lock_guard<std::mutex> lock(data_mutex_);
                in_air_ = in_air;
            });
            RCLCPP_INFO(get_logger(), "Connected to PX4; waiting for armed, airborne state");
            return;
        }
    }

    void controlStep() {
        double pixel_x = 0.0;
        double image_center_x = 0.0;
        double distance_m = 0.0;
        bool target_fresh = false;
        bool vehicle_ready = false;

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            const auto now = std::chrono::steady_clock::now();
            const auto position_age = now - last_position_time_;
            const auto distance_age = now - last_distance_time_;
            const auto pair_age = position_age > distance_age
                ? position_age - distance_age
                : distance_age - position_age;
            target_fresh = have_position_ && have_distance_ && have_camera_info_ &&
                position_age <= target_timeout_ && distance_age <= target_timeout_ &&
                pair_age <= target_timeout_ && distance_m_ <= max_detection_distance_m_;
            vehicle_ready = armed_ && in_air_;
            pixel_x = pixel_x_;
            image_center_x = image_center_x_;
            distance_m = distance_m_;
        }

        if (!vehicle_ready || !target_fresh) {
            stopOffboard();
            if (!vehicle_ready) {
                RCLCPP_INFO_THROTTLE(
                    get_logger(), *get_clock(), 3000,
                    "Follower idle: PX4 must be armed and airborne; target data must be fresh");
            } else {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "Target data stale or out of range; leaving offboard mode");
            }
            return;
        }

        const double horizontal_error = std::clamp(
            (pixel_x - image_center_x) / image_center_x, -1.0, 1.0);
        const double forward_speed = std::clamp(
            forward_gain_ * (distance_m - follow_distance_m_),
            0.0, max_forward_speed_mps_);
        const double lateral_speed = std::clamp(
            lateral_gain_mps_ * horizontal_error,
            -max_lateral_speed_mps_, max_lateral_speed_mps_);

        if (!offboard_active_) {
            offboard_->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});
            const auto result = offboard_->start();
            if (result != mavsdk::Offboard::Result::Success) {
                RCLCPP_ERROR_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "Could not start offboard mode: %s", mavsdk::to_string(result).c_str());
                return;
            }
            offboard_active_ = true;
            RCLCPP_INFO(get_logger(), "Offboard follow control active");
        }

        offboard_->set_velocity_body({
            static_cast<float>(forward_speed),
            static_cast<float>(lateral_speed),
            0.0f,
            0.0f,
        });
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Following target at %.2fm: forward=%.2fm/s lateral=%.2fm/s",
            distance_m, forward_speed, lateral_speed);
    }

    void stopOffboard() {
        if (!offboard_ || !offboard_active_) {
            return;
        }
        offboard_->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});
        const auto result = offboard_->stop();
        if (result != mavsdk::Offboard::Result::Success) {
            RCLCPP_WARN(get_logger(), "Offboard stop returned: %s",
                        mavsdk::to_string(result).c_str());
        }
        offboard_active_ = false;
    }

    mavsdk::Mavsdk mavsdk_;
    std::unique_ptr<mavsdk::Telemetry> telemetry_;
    std::unique_ptr<mavsdk::Offboard> offboard_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr position_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr distance_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::TimerBase::SharedPtr connection_timer_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    std::mutex data_mutex_;
    std::chrono::steady_clock::time_point last_position_time_{};
    std::chrono::steady_clock::time_point last_distance_time_{};
    std::chrono::milliseconds target_timeout_{1500};
    double pixel_x_{0.0};
    double image_center_x_{0.0};
    double distance_m_{0.0};
    double follow_distance_m_{3.0};
    double max_forward_speed_mps_{0.8};
    double max_lateral_speed_mps_{0.5};
    double forward_gain_{0.5};
    double lateral_gain_mps_{0.6};
    double max_detection_distance_m_{30.0};
    bool have_position_{false};
    bool have_distance_{false};
    bool have_camera_info_{false};
    bool armed_{false};
    bool in_air_{false};
    bool offboard_active_{false};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FollowTargetNode>());
    rclcpp::shutdown();
    return 0;
}