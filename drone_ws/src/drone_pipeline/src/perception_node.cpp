// Perception service.
//   in : /camera/image_raw (Image)
//   out: /perception/detection (Detection) - ONLY when something is detected, so no boolean needed.
//
// The Detector interface is the seam: swap FakeDetector for YOLO/TensorRT/OpenCV that reads the image.
// FakeDetector cheats by projecting the hidden ground-truth target into a nadir camera using the
// drone's GPS (/sim/target_truth, /drone/gps). Real detectors only need the frame.
#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <random>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "drone_msgs/msg/detection.hpp"
#include "drone_pipeline/geo.hpp"

using drone_msgs::msg::Detection;
using sensor_msgs::msg::Image;
using sensor_msgs::msg::NavSatFix;

struct Box { double cx, cy, w, h, conf; };

class Detector {
public:
  virtual ~Detector() = default;
  virtual std::optional<Box> detect(const Image & frame) = 0;
};

class FakeDetector : public Detector {
public:
  FakeDetector(double hfov_deg, double prob, double px_sigma, double size_m)
  : hfov_deg_(hfov_deg), prob_(prob), px_sigma_(px_sigma), size_m_(size_m), rng_(42) {}

  void set_drone(const geo::Lla & d) { drone_ = d; }
  void set_target(const geo::Lla & t) { target_ = t; }

  std::optional<Box> detect(const Image & frame) override {
    if (!drone_ || !target_) return std::nullopt;
    const double W = frame.width, H = frame.height;
    const auto rel = geo::lla_to_enu(*drone_, *target_);   // east, north, up
    const double h = -rel[2];                              // height above target
    if (h < 1.0) return std::nullopt;

    // Nadir camera, image "up" = north, pinhole model.
    const double f = (W / 2.0) / std::tan(hfov_deg_ * geo::kDeg2Rad / 2.0);
    std::normal_distribution<double> noise(0.0, px_sigma_);
    const double px = W / 2.0 + f * rel[0] / h + noise(rng_);
    const double py = H / 2.0 - f * rel[1] / h + noise(rng_);
    if (px < 0 || px >= W || py < 0 || py >= H) return std::nullopt;       // out of view
    if (std::uniform_real_distribution<double>(0.0, 1.0)(rng_) > prob_) return std::nullopt;  // missed

    const double size = f * size_m_ / h;
    const double conf = std::uniform_real_distribution<double>(0.6, 0.99)(rng_);
    return Box{px, py, size, size, conf};
  }

private:
  double hfov_deg_, prob_, px_sigma_, size_m_;
  std::optional<geo::Lla> drone_, target_;
  std::mt19937 rng_;
};

class PerceptionNode : public rclcpp::Node {
public:
  PerceptionNode() : Node("perception_node") {
    detector_ = std::make_unique<FakeDetector>(
        declare_parameter("hfov_deg", 90.0), declare_parameter("detect_prob", 0.7),
        declare_parameter("pixel_sigma", 3.0), declare_parameter("target_size_m", 1.0));

    pub_ = create_publisher<Detection>("/perception/detection", 10);

    gps_sub_ = create_subscription<NavSatFix>("/drone/gps", rclcpp::SensorDataQoS(),
        [this](const NavSatFix & m) { detector_->set_drone({m.latitude, m.longitude, m.altitude}); });
    truth_sub_ = create_subscription<NavSatFix>("/sim/target_truth", rclcpp::SensorDataQoS(),
        [this](const NavSatFix & m) { detector_->set_target({m.latitude, m.longitude, m.altitude}); });
    img_sub_ = create_subscription<Image>("/camera/image_raw", rclcpp::SensorDataQoS(),
        [this](Image::ConstSharedPtr img) {
          const auto box = detector_->detect(*img);
          if (!box) return;                                  // nothing detected -> publish nothing
          Detection d;
          d.header = img->header;                            // timestamp = frame capture time
          d.confidence = box->conf;
          d.x = box->cx; d.y = box->cy; d.width = box->w; d.height = box->h;
          d.image_width = img->width; d.image_height = img->height;
          pub_->publish(d);
        });
  }

private:
  std::unique_ptr<FakeDetector> detector_;
  rclcpp::Publisher<Detection>::SharedPtr pub_;
  rclcpp::Subscription<NavSatFix>::SharedPtr gps_sub_, truth_sub_;
  rclcpp::Subscription<Image>::SharedPtr img_sub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PerceptionNode>());
  rclcpp::shutdown();
}
