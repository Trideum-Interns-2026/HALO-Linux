// Movement service.
//   in : /tracking/poi (Poi), /drone/gps (NavSatFix)
//   out: MoveCommand over MAVLink (not ROS). In the sim the "MAVLink link" publishes /sim/mavlink_out.
//
// Behavior: lock onto one POI id, fly (GOTO) to a spot on the orbit circle if far away, then ORBIT it.
// Centering: PX4's orbit keeps the nose pointed at the center. For a gimbal/yaw-only centering
// loop, add that here using the POI bearing.
#include <cmath>
#include <memory>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "drone_msgs/msg/move_command.hpp"
#include "drone_msgs/msg/poi.hpp"
#include "drone_pipeline/geo.hpp"

using drone_msgs::msg::MoveCommand;
using drone_msgs::msg::Poi;
using sensor_msgs::msg::NavSatFix;

// The seam to PX4. Implement this with MAVSDK (Action: goto_location / do_orbit) or raw MAVLink
// (MAV_CMD_DO_REPOSITION, MAV_CMD_DO_ORBIT) and the rest of the node does not change.
// Check the altitude frame: MAVLink commands usually want AMSL, /drone/gps here is ellipsoid height.
class MavlinkLink {
public:
  virtual ~MavlinkLink() = default;
  virtual void send(const MoveCommand & cmd) = 0;
};

class SimMavlinkLink : public MavlinkLink {
public:
  explicit SimMavlinkLink(rclcpp::Node & node)
  : pub_(node.create_publisher<MoveCommand>("/sim/mavlink_out", 10)) {}
  void send(const MoveCommand & cmd) override { pub_->publish(cmd); }

private:
  rclcpp::Publisher<MoveCommand>::SharedPtr pub_;
};

class MovementNode : public rclcpp::Node {
public:
  MovementNode() : Node("movement_node") {
    radius_ = declare_parameter("orbit_radius_m", 15.0);
    alt_ = declare_parameter("orbit_alt_m", 30.0);
    resend_dist_ = declare_parameter("resend_distance_m", 2.0);
    lock_timeout_ = declare_parameter("lock_timeout_s", 5.0);

    link_ = std::make_unique<SimMavlinkLink>(*this);
    gps_sub_ = create_subscription<NavSatFix>("/drone/gps", rclcpp::SensorDataQoS(),
        [this](const NavSatFix & m) { drone_ = geo::Lla{m.latitude, m.longitude, m.altitude}; });
    poi_sub_ = create_subscription<Poi>("/tracking/poi", 10, [this](const Poi & p) { on_poi(p); });
  }

private:
  void on_poi(const Poi & poi) {
    const double t = now().seconds();
    if (locked_id_ == 0 || t - locked_seen_ > lock_timeout_) {   // pick a new target
      locked_id_ = poi.id;
      last_sent_.reset();
    }
    if (poi.id != locked_id_) return;
    locked_seen_ = t;

    const geo::Lla target{poi.lla[0], poi.lla[1], poi.lla[2]};
    if (last_sent_ && t - last_sent_t_ < 2.0 &&
        geo::distance_m(geo::lla_to_ecef(*last_sent_), geo::lla_to_ecef(target)) < resend_dist_) {
      return;                                                     // target barely moved, don't spam
    }

    MoveCommand cmd;
    cmd.header.stamp = now();
    cmd.alt = alt_;

    // Drone position relative to POI decides GOTO vs ORBIT.
    double ux = 1.0, uy = 0.0, dist = 0.0;
    if (drone_) {
      const auto rel = geo::lla_to_enu(target, *drone_);
      dist = std::hypot(rel[0], rel[1]);
      if (dist > 1e-3) { ux = rel[0] / dist; uy = rel[1] / dist; }
    }

    if (drone_ && dist > 2.0 * radius_) {
      const geo::Lla spot = geo::offset_lla(target, ux * radius_, uy * radius_, 0.0);
      cmd.type = MoveCommand::GOTO;
      cmd.lat = spot.lat; cmd.lon = spot.lon; cmd.radius = 0.0;
    } else {
      cmd.type = MoveCommand::ORBIT;
      cmd.lat = target.lat; cmd.lon = target.lon; cmd.radius = radius_;
    }
    link_->send(cmd);
    last_sent_ = target;
    last_sent_t_ = t;
  }

  double radius_, alt_, resend_dist_, lock_timeout_;
  uint32_t locked_id_ = 0;
  double locked_seen_ = 0.0, last_sent_t_ = 0.0;
  std::optional<geo::Lla> drone_, last_sent_;
  std::unique_ptr<MavlinkLink> link_;
  rclcpp::Subscription<NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<Poi>::SharedPtr poi_sub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MovementNode>());
  rclcpp::shutdown();
}
