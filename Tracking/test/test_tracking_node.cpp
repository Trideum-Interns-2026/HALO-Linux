#include <chrono>
#include <memory>
#include <thread>

#include <geolocation_msgs/msg/geolocation_state.hpp>
#include <gtest/gtest.h>
#include <perception_msgs/msg/detection.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tracking_msgs/msg/tracking_state.hpp>

#include "tracking/tracking_node.hpp"

class TrackingNodeTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        int argc = 0;
        char** argv = nullptr;
        rclcpp::init(argc, argv);
    }

    static void TearDownTestSuite() {
        rclcpp::shutdown();
    }
};

TEST_F(TrackingNodeTest, PublishesPositionWithDetectionHeaderAndUsesDefaults) {
    auto node = std::make_shared<TrackingNode>();
    auto observer = std::make_shared<rclcpp::Node>("tracking_output_observer");
    tracking_msgs::msg::TrackingState::SharedPtr received;
    auto subscription = observer->create_subscription<tracking_msgs::msg::TrackingState>(
        "/tracking/position", rclcpp::QoS(10),
        [&received](tracking_msgs::msg::TrackingState::SharedPtr message) {
            received = std::move(message);
        });
    (void)subscription;
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(observer);

    auto camera = std::make_shared<sensor_msgs::msg::CameraInfo>();
    camera->k[0] = 500.0;
    camera->k[2] = 320.0;
    camera->k[4] = 500.0;
    camera->k[5] = 240.0;
    node->onCameraInfo(camera);

    auto position = std::make_shared<geolocation_msgs::msg::GeolocationState>();
    position->header.stamp = node->now();
    position->latitude_deg = 0.0;
    position->longitude_deg = 0.0;
    position->altitude_m = 100.0;
    position->error_x_m = 0.3;
    position->error_y_m = 0.3;
    position->error_z_m = 0.3;
    node->onGeolocation(position);

    auto attitude = std::make_shared<px4_msgs::msg::VehicleAttitude>();
    attitude->q = {1.0F, 0.0F, 0.0F, 0.0F};
    node->onAttitude(attitude);

    auto detection = std::make_shared<perception_msgs::msg::Detection>();
    detection->header.stamp = node->now();
    detection->header.frame_id = "camera_optical_frame";
    detection->confidence = 0.9F;
    detection->x_min = 270.0F;
    detection->y_min = 190.0F;
    detection->x_max = 370.0F;
    detection->y_max = 290.0F;
    detection->confidence = 0.1F;
    node->onDetection(detection);
    executor.spin_some();
    EXPECT_FALSE(received);

    detection->confidence = 0.9F;
    node->onDetection(detection);

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!received && std::chrono::steady_clock::now() < deadline) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    ASSERT_TRUE(received);
    EXPECT_EQ(received->header.frame_id, "camera_optical_frame");
    EXPECT_EQ(received->header.stamp.sec, detection->header.stamp.sec);
    EXPECT_EQ(received->header.stamp.nanosec, detection->header.stamp.nanosec);
    EXPECT_GT(received->latitude_deg, position->latitude_deg);
    EXPECT_NEAR(received->longitude_deg, position->longitude_deg, 1e-8);
    EXPECT_GT(received->error_x_m, 0.0);
    EXPECT_GT(received->error_y_m, 0.0);
    EXPECT_GT(received->error_z_m, 0.0);
    EXPECT_EQ(node->get_parameter("detection_topic").as_string(), "/perception/detection");
    EXPECT_EQ(node->get_parameter("vehicle_attitude_topic").as_string(),
        "/fmu/out/vehicle_attitude");
}
