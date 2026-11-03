// SIM ONLY. Publishes blank 640x480 mono frames at 10 Hz so perception has a stream to consume.
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

using namespace std::chrono_literals;

class CameraSim : public rclcpp::Node {
public:
  CameraSim() : Node("camera_sim") {
    width_ = declare_parameter("image_width", 640);
    height_ = declare_parameter("image_height", 480);
    pub_ = create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", rclcpp::SensorDataQoS());
    timer_ = create_wall_timer(100ms, [this] {
      sensor_msgs::msg::Image img;
      img.header.stamp = now();
      img.header.frame_id = "camera";
      img.width = static_cast<uint32_t>(width_);
      img.height = static_cast<uint32_t>(height_);
      img.encoding = "mono8";
      img.step = img.width;
      img.data.assign(static_cast<size_t>(width_) * height_, 0);
      pub_->publish(img);
    });
  }

private:
  int width_, height_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraSim>());
  rclcpp::shutdown();
}
