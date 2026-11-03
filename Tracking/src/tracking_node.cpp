#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "tracking/target_kalman_filter.hpp"

class TrackingNode : public rclcpp::Node {
public:
    TrackingNode()
        : Node("tracking_node") {
        declare_parameter("input_detect_topic", "/tracking/detection/position");
        declare_parameter("camera_info_topic", "/camera/front/camera_info");
        declare_parameter("output_fix_topic", "/tracking/fix");
        declare_parameter("output_covariance_topic", "/tracking/error");
        declare_parameter("origin_latitude_deg", 0.0);
        declare_parameter("origin_longitude_deg", 0.0);
        declare_parameter("origin_altitude_m", 0.0);
        declare_parameter("camera_height_m", 1.75);
        declare_parameter("camera_height_stddev_m", 0.10);
        declare_parameter("process_noise_stddev_m", 0.50);

        human_height_m_ = get_parameter("camera_height_m").as_double();
        human_height_stddev_m_ = get_parameter("camera_height_stddev_m").as_double();
        process_noise_stddev_m_ = get_parameter("process_noise_stddev_m").as_double();
        filter_ = std::make_unique<TargetKalmanFilter>(human_height_m_, human_height_stddev_m_, process_noise_stddev_m_);

        detection_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
            get_parameter("input_detect_topic").as_string(), 10,
            std::bind(&TrackingNode::detectionCallback, this, std::placeholders::_1));

        camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            get_parameter("camera_info_topic").as_string(), 10,
            std::bind(&TrackingNode::cameraInfoCallback, this, std::placeholders::_1));

        output_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(
            get_parameter("output_fix_topic").as_string(), 10);
        covariance_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            get_parameter("output_covariance_topic").as_string(), 10);
    }

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        fx_ = msg->k[0];
        fy_ = msg->k[4];
        cx_ = msg->k[2];
        cy_ = msg->k[5];
        camera_info_ready_ = std::isfinite(fx_) && std::isfinite(fy_) && fx_ > 0.0 && fy_ > 0.0;
    }

    void detectionCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        if (msg == nullptr) {
            return;
        }

        double focal_length_px = 0.0;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!camera_info_ready_) {
                return;
            }
            focal_length_px = fx_;
            if (msg->point.z <= 0.0) {
                return;
            }
            const auto now = this->now();
            if (last_update_time_.seconds() > 0.0) {
                const double dt = (now - last_update_time_).seconds();
                if (dt > 0.0) {
                    filter_->predict(dt);
                }
            }
            last_update_time_ = now;
            filter_->updateFromImage(msg->point.x, msg->point.y, msg->point.z, focal_length_px, cx_, cy_, 0.0);
            const auto estimate = filter_->estimate(
                get_parameter("origin_latitude_deg").as_double(),
                get_parameter("origin_longitude_deg").as_double(),
                get_parameter("origin_altitude_m").as_double());

            sensor_msgs::msg::NavSatFix fix;
            fix.header.stamp = now;
            fix.header.frame_id = "earth";
            fix.latitude = estimate.latitude_deg;
            fix.longitude = estimate.longitude_deg;
            fix.altitude = estimate.altitude_m;
            fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
            fix.position_covariance[0] = estimate.covariance_enu[0];
            fix.position_covariance[4] = estimate.covariance_enu[4];
            fix.position_covariance[8] = estimate.covariance_enu[8];
            output_pub_->publish(fix);

            geometry_msgs::msg::PoseWithCovarianceStamped covariance_msg;
            covariance_msg.header = fix.header;
            covariance_msg.pose.pose.position.x = estimate.local_enu_m[0];
            covariance_msg.pose.pose.position.y = estimate.local_enu_m[1];
            covariance_msg.pose.pose.position.z = estimate.local_enu_m[2];
            covariance_msg.pose.covariance[0] = estimate.covariance_enu[0];
            covariance_msg.pose.covariance[7] = estimate.covariance_enu[4];
            covariance_msg.pose.covariance[14] = estimate.covariance_enu[8];
            covariance_pub_->publish(covariance_msg);
        }
    }

private:
    std::mutex mutex_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr detection_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr output_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr covariance_pub_;

    std::unique_ptr<TargetKalmanFilter> filter_;
    double human_height_m_{1.75};
    double human_height_stddev_m_{0.1};
    double process_noise_stddev_m_{0.5};
    double fx_{0.0};
    double fy_{0.0};
    double cx_{0.0};
    double cy_{0.0};
    bool camera_info_ready_{false};
    rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrackingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
