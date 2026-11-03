// Geolocation service.
//   in : /perception/detection (Detection), /drone/gps (NavSatFix)
//   out: /geolocation/estimate (GeoEstimate) = LLA triplet + ECEF error triplet
//
// Ranging happens here and is not published: assume the target is on the ground, so range =
// height above ground, then invert the pinhole model. Camera is assumed nadir (straight down),
// image-up = north. Add drone attitude (yaw/pitch/roll) before using this on a real airframe.
// Time sync: uses the latest GPS fix. For real flights, buffer fixes and interpolate to the detection stamp.
#include <cmath>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "drone_msgs/msg/detection.hpp"
#include "drone_msgs/msg/geo_estimate.hpp"
#include "drone_pipeline/geo.hpp"

using drone_msgs::msg::Detection;
using drone_msgs::msg::GeoEstimate;
using sensor_msgs::msg::NavSatFix;

class GeolocationNode : public rclcpp::Node {
public:
  GeolocationNode() : Node("geolocation_node") {
    hfov_deg_ = declare_parameter("hfov_deg", 90.0);
    ground_alt_ = declare_parameter("ground_alt_m", 0.0);
    pixel_sigma_ = declare_parameter("pixel_sigma", 3.0);
    height_sigma_ = declare_parameter("height_sigma_m", 1.0);   // covers "average human height"

    pub_ = create_publisher<GeoEstimate>("/geolocation/estimate", 10);
    gps_sub_ = create_subscription<NavSatFix>("/drone/gps", rclcpp::SensorDataQoS(),
        [this](const NavSatFix & m) { gps_ = m; gps_t_ = now().seconds(); });
    det_sub_ = create_subscription<Detection>("/perception/detection", 10,
        [this](const Detection & d) { on_detection(d); });
  }

private:
  void on_detection(const Detection & d) {
    if (!gps_ || now().seconds() - gps_t_ > 1.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No fresh GPS fix, dropping detection");
      return;
    }
    const geo::Lla drone{gps_->latitude, gps_->longitude, gps_->altitude};
    const double h = drone.alt - ground_alt_;
    if (h < 1.0) return;

    const double f = (d.image_width / 2.0) / std::tan(hfov_deg_ * geo::kDeg2Rad / 2.0);
    const double east = (d.x - d.image_width / 2.0) * h / f;
    const double north = -(d.y - d.image_height / 2.0) * h / f;
    const geo::Lla target = geo::offset_lla(drone, east, north, -h);     // lands at ground level

    const bool known = gps_->position_covariance_type != NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    const double gps_h = known ? std::sqrt(gps_->position_covariance[0]) : 2.0;
    const double gps_v = known ? std::sqrt(gps_->position_covariance[8]) : 3.0;
    const double sig_px = h * pixel_sigma_ / f;                          // pixel noise projected to ground
    const double sig_h = std::hypot(sig_px, gps_h);
    const double sig_v = std::hypot(height_sigma_, gps_v);
    const geo::Vec3 err = geo::enu_sigma_to_ecef_sigma(target, {sig_h, sig_h, sig_v});

    GeoEstimate e;
    e.header.stamp = d.header.stamp;
    e.header.frame_id = "wgs84";
    e.confidence = d.confidence;
    e.lla = {target.lat, target.lon, target.alt};
    e.error_ecef = {err[0], err[1], err[2]};
    pub_->publish(e);
  }

  double hfov_deg_, ground_alt_, pixel_sigma_, height_sigma_, gps_t_ = 0.0;
  std::optional<NavSatFix> gps_;
  rclcpp::Publisher<GeoEstimate>::SharedPtr pub_;
  rclcpp::Subscription<NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<Detection>::SharedPtr det_sub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GeolocationNode>());
  rclcpp::shutdown();
}
