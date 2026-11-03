// POI tracking service.
//   in : /geolocation/estimate (GeoEstimate)
//   out: /tracking/poi (Poi) - one message per track update, with a stable id
// Nearest-neighbor association in ECEF (Euclidean, so distances are real meters) + exponential smoothing.
#include <algorithm>
#include <cmath>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "drone_msgs/msg/geo_estimate.hpp"
#include "drone_msgs/msg/poi.hpp"
#include "drone_pipeline/geo.hpp"

using namespace std::chrono_literals;
using drone_msgs::msg::GeoEstimate;
using drone_msgs::msg::Poi;

struct Track {
  uint32_t id;
  geo::Vec3 ecef;
  double conf;
  uint32_t hits;
  double last_seen;
};

class PoiTrackerNode : public rclcpp::Node {
public:
  PoiTrackerNode() : Node("poi_tracker_node") {
    gate_m_ = declare_parameter("gate_m", 8.0);
    alpha_ = declare_parameter("alpha", 0.3);
    timeout_s_ = declare_parameter("track_timeout_s", 10.0);

    pub_ = create_publisher<Poi>("/tracking/poi", 10);
    sub_ = create_subscription<GeoEstimate>("/geolocation/estimate", 10,
        [this](const GeoEstimate & e) { on_estimate(e); });
    gc_ = create_wall_timer(1s, [this] {
      const double t = now().seconds();
      tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
          [&](const Track & tr) { return t - tr.last_seen > timeout_s_; }), tracks_.end());
    });
  }

private:
  void on_estimate(const GeoEstimate & e) {
    const double t = now().seconds();
    const geo::Vec3 m = geo::lla_to_ecef({e.lla[0], e.lla[1], e.lla[2]});

    Track * best = nullptr;
    double best_d = gate_m_;
    for (auto & tr : tracks_) {
      const double d = geo::distance_m(tr.ecef, m);
      if (d < best_d) { best = &tr; best_d = d; }
    }
    if (!best) {
      tracks_.push_back(Track{next_id_++, m, e.confidence, 1, t});
      best = &tracks_.back();
    } else {
      for (int i = 0; i < 3; ++i) best->ecef[i] += alpha_ * (m[i] - best->ecef[i]);
      best->conf = 0.8 * best->conf + 0.2 * e.confidence;
      best->hits++;
      best->last_seen = t;
    }

    // Heuristic: error shrinks as measurements are averaged, floored by the smoothing window.
    const double shrink = 1.0 / std::sqrt(static_cast<double>(std::min<uint32_t>(best->hits, 6)));
    const geo::Lla p = geo::ecef_to_lla(best->ecef);

    Poi out;
    out.header.stamp = e.header.stamp;
    out.header.frame_id = "wgs84";
    out.id = best->id;
    out.confidence = best->conf;
    out.hits = best->hits;
    out.lla = {p.lat, p.lon, p.alt};
    out.error_ecef = {e.error_ecef[0] * shrink, e.error_ecef[1] * shrink, e.error_ecef[2] * shrink};
    pub_->publish(out);
  }

  double gate_m_, alpha_, timeout_s_;
  uint32_t next_id_ = 1;
  std::vector<Track> tracks_;
  rclcpp::Publisher<Poi>::SharedPtr pub_;
  rclcpp::Subscription<GeoEstimate>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr gc_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoiTrackerNode>());
  rclcpp::shutdown();
}
