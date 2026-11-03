#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "geolocation/drone_gps_kalman_filter.hpp"

class GeolocationNode : public rclcpp::Node {
public:
    GeolocationNode()
        : Node("geolocation_node"),
          filter_(1.0, 1.0, 0.25) {
        declare_parameter("input_fix_topic", "/gps/fix");
        declare_parameter("output_fix_topic", "/geolocation/fix");
        declare_parameter("output_covariance_topic", "/geolocation/error");
        declare_parameter("horizontal_stddev_m", 1.0);
        declare_parameter("vertical_stddev_m", 1.0);
        declare_parameter("process_noise_variance_m2_per_s", 0.25);

        const double horizontal_stddev = get_parameter("horizontal_stddev_m").as_double();
        const double vertical_stddev = get_parameter("vertical_stddev_m").as_double();
        const double process_noise = get_parameter("process_noise_variance_m2_per_s").as_double();
        filter_ = DroneGpsKalmanFilter(horizontal_stddev, vertical_stddev, process_noise);

        input_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
            get_parameter("input_fix_topic").as_string(), 10,
            std::bind(&GeolocationNode::gpsCallback, this, std::placeholders::_1));

        output_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(
            get_parameter("output_fix_topic").as_string(), 10);
        covariance_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            get_parameter("output_covariance_topic").as_string(), 10);
    }

    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        if (!std::isfinite(msg->latitude) || !std::isfinite(msg->longitude) || !std::isfinite(msg->altitude)) {
            return;
        }

        const auto now = this->now();
        std::lock_guard<std::mutex> lock(mutex_);
        const double dt = last_measurement_time_.seconds() > 0.0 ?
            (now - last_measurement_time_).seconds() : 0.0;
        last_measurement_time_ = now;

        const auto estimate = filter_.update(msg->latitude, msg->longitude, msg->altitude, dt);
        sensor_msgs::msg::NavSatFix fixed;
        fixed.header.stamp = now;
        fixed.header.frame_id = "earth";
        fixed.latitude = estimate.latitude_deg;
        fixed.longitude = estimate.longitude_deg;
        fixed.altitude = estimate.altitude_m;
        fixed.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
        fixed.position_covariance[0] = estimate.horizontal_radius_3sigma_m * estimate.horizontal_radius_3sigma_m;
        fixed.position_covariance[4] = estimate.horizontal_radius_3sigma_m * estimate.horizontal_radius_3sigma_m;
        fixed.position_covariance[8] = estimate.vertical_3sigma_m * estimate.vertical_3sigma_m;
        output_pub_->publish(fixed);

        geometry_msgs::msg::PoseWithCovarianceStamped covariance_msg;
        covariance_msg.header = fixed.header;
        covariance_msg.pose.pose.position.x = estimate.enu_m[0];
        covariance_msg.pose.pose.position.y = estimate.enu_m[1];
        covariance_msg.pose.pose.position.z = estimate.enu_m[2];
        covariance_msg.pose.covariance[0] = estimate.covariance_enu[0];
        covariance_msg.pose.covariance[7] = estimate.covariance_enu[4];
        covariance_msg.pose.covariance[14] = estimate.covariance_enu[8];
        covariance_pub_->publish(covariance_msg);
    }

private:
    std::mutex mutex_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr input_sub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr output_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr covariance_pub_;
    DroneGpsKalmanFilter filter_;
    rclcpp::Time last_measurement_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GeolocationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
