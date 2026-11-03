// SIM ONLY. Publishes the hidden ground-truth position of a person walking in a slow circle.
// Only perception_node's FakeDetector reads this. Delete this node when you have a real camera.
#include <chrono>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "drone_pipeline/geo.hpp"

using namespace std::chrono_literals;

class WorldSim : public rclcpp::Node {
public:
  WorldSim() : Node("world_sim") {
    origin_ = {declare_parameter("origin_lat", 33.7756), declare_parameter("origin_lon", -84.3963), 0.0};
    walk_radius_ = declare_parameter("walk_radius_m", 25.0);
    walk_period_ = declare_parameter("walk_period_s", 120.0);
    pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("/sim/target_truth", rclcpp::SensorDataQoS());
    t0_ = now().seconds();
    timer_ = create_wall_timer(100ms, [this] {
      const double th = 2.0 * 3.14159265358979 * (now().seconds() - t0_) / walk_period_;
      const geo::Lla p = geo::offset_lla(origin_, walk_radius_ * std::cos(th), walk_radius_ * std::sin(th), 0.0);
      sensor_msgs::msg::NavSatFix m;
      m.header.stamp = now();
      m.header.frame_id = "wgs84";
      m.latitude = p.lat; m.longitude = p.lon; m.altitude = p.alt;
      pub_->publish(m);
    });
  }

private:
  geo::Lla origin_{};
  double walk_radius_, walk_period_, t0_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WorldSim>());
  rclcpp::shutdown();
}
