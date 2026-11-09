// Movement node: Lisa's follow-person controller as a service. It turns the person's position
// in the camera image into body-frame velocity commands (offboard) that keep the person
// centred at a set distance, and scans for them when they are lost. It never arms or takes
// off unless `auto_arm_takeoff` is set, and it returns PX4 to Hold on exit.
#pragma once

#include <memory>
#include <mutex>
#include <optional>

#include <perception_msgs/msg/detection.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include "movement/follow_logic.hpp"
#include "movement/vehicle.hpp"

namespace movement {

constexpr auto kLoopPeriod = std::chrono::milliseconds(100);
constexpr int kDefaultTargetTimeoutMs = 800;

class MovementNode : public rclcpp::Node {
public:
    explicit MovementNode(
        std::unique_ptr<VehicleInterface> vehicle,
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~MovementNode() override;

    // Subscription entry points, public so tests can feed messages without an executor.
    void onDetection(const perception_msgs::msg::Detection& msg, const rclcpp::Time& received);
    void onCameraInfo(const sensor_msgs::msg::CameraInfo& msg);

    // One control step (the original's controlLoop). Called by the timer.
    void tick(const rclcpp::Time& now);

    // Stops the timer and leaves offboard (PX4 goes to Hold). Safe to call more than once.
    void shutdown();

    [[nodiscard]] bool offboardActive() const { return offboard_started_; }
    [[nodiscard]] bool searching() const { return search_active_; }

private:
    void armAndTakeoffIfNeeded(const VehicleState& state);

    std::unique_ptr<VehicleInterface> vehicle_;
    FollowParams params_;
    OrbitParams orbit_;
    OrbitState orbit_state_;
    CommandLimiter limiter_;
    double last_offset_{0.0};     // normalised offset of the target when last seen
    double last_turn_sign_{1.0};  // sign of the yaw command when last seen (+1 = right)
    int target_timeout_ms_{kDefaultTargetTimeoutMs};
    double person_height_m_{kDefaultTargetPersonHeightM};
    bool auto_arm_takeoff_{false};
    double search_timeout_s_{0.0};  // 0 = scan until the target returns, as the original did

    rclcpp::Subscription<perception_msgs::msg::Detection>::SharedPtr detection_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    std::mutex state_mutex_;
    struct Target {
        double x_px{0.0};
        double y_min_px{0.0};
        double y_max_px{0.0};
    } latest_target_;
    CameraGeometry camera_;
    bool have_position_{false};
    bool have_camera_info_{false};
    rclcpp::Time last_detection_time_{0, 0, RCL_ROS_TIME};

    bool offboard_started_{false};
    bool search_active_{false};
    bool search_gave_up_{false};
    rclcpp::Time search_start_{0, 0, RCL_ROS_TIME};
};

}  // namespace movement
