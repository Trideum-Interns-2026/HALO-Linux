// SIM ONLY. Stands in for PX4 + the flight controller.
//   in : /sim/mavlink_out (MoveCommand)  - what movement_node "sent over MAVLink"
//   out: /drone/gps (NavSatFix, noisy), /drone/battery (BatteryState)
// Replace with the real thing: GPS/battery come from PX4 (via uXRCE-DDS or MAVLink telemetry).
#include <algorithm>
#include <chrono>
#include <cmath>
#include <random>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "drone_msgs/msg/move_command.hpp"
#include "drone_pipeline/geo.hpp"

using namespace std::chrono_literals;
using drone_msgs::msg::MoveCommand;
using sensor_msgs::msg::NavSatFix;

class Px4Sim : public rclcpp::Node {
  enum class Mode { kHold, kGoto, kOrbit };

public:
  Px4Sim() : Node("px4_sim"), rng_(7) {
    const double olat = declare_parameter("origin_lat", 33.7756);
    const double olon = declare_parameter("origin_lon", -84.3963);
    const double e0 = declare_parameter("start_east_m", -40.0);
    const double n0 = declare_parameter("start_north_m", -30.0);
    alt_ = goal_alt_ = declare_parameter("start_alt_m", 30.0);
    speed_ = declare_parameter("speed_mps", 5.0);

    pos_ = geo::offset_lla({olat, olon, 0.0}, e0, n0, 0.0);
    pos_.alt = alt_;
    t0_ = now().seconds();

    gps_pub_ = create_publisher<NavSatFix>("/drone/gps", rclcpp::SensorDataQoS());
    bat_pub_ = create_publisher<sensor_msgs::msg::BatteryState>("/drone/battery", rclcpp::SensorDataQoS());
    cmd_sub_ = create_subscription<MoveCommand>("/sim/mavlink_out", 10,
        [this](const MoveCommand & c) { on_command(c); });

    step_timer_ = create_wall_timer(50ms, [this] { step(); });   // 20 Hz physics
    bat_timer_ = create_wall_timer(1s, [this] { publish_battery(); });
  }

private:
  void on_command(const MoveCommand & c) {
    goal_ = {c.lat, c.lon, 0.0};
    goal_alt_ = c.alt;
    if (c.type == MoveCommand::ORBIT) {
      radius_ = std::max(c.radius, 3.0);
      mode_ = Mode::kOrbit;
    } else {
      mode_ = Mode::kGoto;
    }
  }

  void step() {
    const double dt = 0.05;
    alt_ += std::clamp(goal_alt_ - alt_, -2.0 * dt, 2.0 * dt);   // 2 m/s climb

    if (mode_ == Mode::kGoto) {
      const auto d = geo::lla_to_enu(pos_, goal_);
      const double dist = std::hypot(d[0], d[1]);
      if (dist < 1.0) {
        mode_ = Mode::kHold;
      } else {
        const double s = std::min(speed_ * dt, dist);
        pos_ = geo::offset_lla(pos_, d[0] / dist * s, d[1] / dist * s, 0.0);
      }
    } else if (mode_ == Mode::kOrbit) {
      const auto rel = geo::lla_to_enu(goal_, pos_);          // drone relative to orbit center
      const double r = std::hypot(rel[0], rel[1]);
      if (r > radius_ + 1.0) {                                 // fly in toward the circle
        const double s = std::min(speed_ * dt, r - radius_);
        pos_ = geo::offset_lla(pos_, -rel[0] / r * s, -rel[1] / r * s, 0.0);
      } else {                                                 // ride the circle, counter-clockwise
        const double th = std::atan2(rel[1], rel[0]) + speed_ / radius_ * dt;
        pos_ = geo::offset_lla(goal_, radius_ * std::cos(th), radius_ * std::sin(th), 0.0);
      }
    }
    pos_.alt = alt_;

    if (++tick_ % 4 == 0) publish_gps();                       // 5 Hz GPS
  }

  void publish_gps() {
    constexpr double kSigH = 0.5, kSigV = 0.8;
    const geo::Lla n = geo::offset_lla(pos_, kSigH * unit_(rng_), kSigH * unit_(rng_), kSigV * unit_(rng_));
    NavSatFix m;
    m.header.stamp = now();
    m.header.frame_id = "wgs84";
    m.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    m.latitude = n.lat; m.longitude = n.lon; m.altitude = n.alt;
    m.position_covariance_type = NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    m.position_covariance[0] = m.position_covariance[4] = kSigH * kSigH;
    m.position_covariance[8] = kSigV * kSigV;
    gps_pub_->publish(m);
  }

  void publish_battery() {
    sensor_msgs::msg::BatteryState b;
    b.header.stamp = now();
    b.percentage = static_cast<float>(std::max(0.05, 1.0 - 0.001 * (now().seconds() - t0_)));
    b.voltage = static_cast<float>(12.0 + 4.8 * b.percentage);
    bat_pub_->publish(b);
  }

  std::mt19937 rng_;
  std::normal_distribution<double> unit_{0.0, 1.0};
  Mode mode_ = Mode::kHold;
  geo::Lla pos_{}, goal_{};
  double alt_, goal_alt_, radius_ = 15.0, speed_, t0_;
  uint64_t tick_ = 0;
  rclcpp::Publisher<NavSatFix>::SharedPtr gps_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr bat_pub_;
  rclcpp::Subscription<MoveCommand>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr step_timer_, bat_timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Px4Sim>());
  rclcpp::shutdown();
}
