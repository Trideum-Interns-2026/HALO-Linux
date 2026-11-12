// Movement node: Lisa's follow-person controller as a service. It turns the tracked person's
// position (plus the drone's position and heading) into body-frame velocity commands (offboard) that keep the person
// centred at a set distance, and scans for them when they are lost. It never arms or takes
// off unless `auto_arm_takeoff` is set, and it returns PX4 to Hold on exit.
#pragma once

#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <geolocation_msgs/msg/geolocation_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tracking_msgs/msg/tracking_state.hpp>

#include "movement/follow_logic.hpp"
#include "movement/heading_freshness.hpp"
#include "movement/target_geometry.hpp"
#include "movement/vehicle.hpp"

namespace movement {

constexpr auto kLoopPeriod = std::chrono::milliseconds(100);
constexpr int kDefaultTargetTimeoutMs = 800;
// Largest 3-sigma ECEF error norm (m) the gate can be set to; 0 = gate off (the default).
constexpr double kDefaultMaxTargetErrorM = 0.0;
constexpr double kMaxTargetErrorCeilingM = 1000.0;

class MovementNode : public rclcpp::Node {
public:
    explicit MovementNode(
        std::unique_ptr<VehicleInterface> vehicle,
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions(),
        const std::string& mavsdk_url = std::string(),
        double heading_timeout_s = kDefaultHeadingTimeoutS);  // the last two only appear in the startup log line
    ~MovementNode() override;

    // Subscription entry points, public so tests can feed messages without an executor.
    // `received` is the node time of receipt; freshness never uses the message header stamp.
    void onTracking(const tracking_msgs::msg::TrackingState& msg, const rclcpp::Time& received);
    void onGeolocation(const geolocation_msgs::msg::GeolocationState& msg,
                       const rclcpp::Time& received);
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
    double max_target_error_m_{kDefaultMaxTargetErrorM};  // 0 = off
    bool auto_arm_takeoff_{false};
    double search_timeout_s_{0.0};  // 0 = scan until the target returns, as the original did

    rclcpp::Subscription<tracking_msgs::msg::TrackingState>::SharedPtr tracking_sub_;
    rclcpp::Subscription<geolocation_msgs::msg::GeolocationState>::SharedPtr geolocation_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    std::mutex state_mutex_;
    GeoPoint person_;  // latest tracked person position
    GeoPoint drone_;   // latest drone position
    CameraGeometry camera_;
    bool have_person_{false};
    bool have_drone_{false};
    bool have_camera_info_{false};
    rclcpp::Time last_tracking_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_geolocation_time_{0, 0, RCL_ROS_TIME};

    bool offboard_started_{false};
    bool search_active_{false};
    bool search_gave_up_{false};
    rclcpp::Time search_start_{0, 0, RCL_ROS_TIME};
};

}  // namespace movement
