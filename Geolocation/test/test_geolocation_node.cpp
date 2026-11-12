#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

#include <geolocation_msgs/msg/geolocation_state.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>

#include "geolocation/geolocation_node.hpp"

class GeolocationNodeTest : public ::testing::Test {
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

TEST_F(GeolocationNodeTest, PublishesNothingForNoFixOrNonFiniteInput) {
    auto node = std::make_shared<GeolocationNode>();
    auto observer = std::make_shared<rclcpp::Node>("geolocation_silent_observer");
    int received = 0;
    auto subscription = observer->create_subscription<geolocation_msgs::msg::GeolocationState>(
        "/geolocation/position", rclcpp::QoS(10),
        [&received](geolocation_msgs::msg::GeolocationState::SharedPtr) { ++received; });
    (void)subscription;
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(observer);

    auto no_fix = std::make_shared<sensor_msgs::msg::NavSatFix>();
    no_fix->status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
    no_fix->latitude = 34.7;
    no_fix->longitude = -86.6;
    no_fix->altitude = 200.0;
    node->onGps(no_fix);

    auto not_finite = std::make_shared<sensor_msgs::msg::NavSatFix>(*no_fix);
    not_finite->status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    not_finite->latitude = std::nan("");
    EXPECT_NO_THROW(node->onGps(not_finite));
    EXPECT_NO_THROW(node->onGps(nullptr));

    for (int i = 0; i < 20; ++i) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    EXPECT_EQ(received, 0);
}

TEST_F(GeolocationNodeTest, CopiesFixHeaderAndPublishesEstimate) {
    auto node = std::make_shared<GeolocationNode>();
    auto observer = std::make_shared<rclcpp::Node>("geolocation_output_observer");
    geolocation_msgs::msg::GeolocationState::SharedPtr received;
    auto subscription = observer->create_subscription<geolocation_msgs::msg::GeolocationState>(
        "/geolocation/position", rclcpp::QoS(10),
        [&received](geolocation_msgs::msg::GeolocationState::SharedPtr message) {
            received = std::move(message);
        });
    (void)subscription;
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(observer);

    auto fix = std::make_shared<sensor_msgs::msg::NavSatFix>();
    fix->header.stamp = node->now();
    fix->header.frame_id = "gps_frame";
    fix->status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    fix->latitude = 34.7;
    fix->longitude = -86.6;
    fix->altitude = 200.0;
    node->onGps(fix);

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!received && std::chrono::steady_clock::now() < deadline) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    ASSERT_TRUE(received);
    EXPECT_EQ(received->header.frame_id, "gps_frame");
    EXPECT_EQ(received->header.stamp.sec, fix->header.stamp.sec);
    EXPECT_EQ(received->header.stamp.nanosec, fix->header.stamp.nanosec);
    EXPECT_NEAR(received->latitude_deg, fix->latitude, 1e-10);
    EXPECT_NEAR(received->longitude_deg, fix->longitude, 1e-10);
    EXPECT_NEAR(received->altitude_m, fix->altitude, 1e-4);
    EXPECT_GT(received->error_x_m, 0.0);
    EXPECT_GT(received->error_y_m, 0.0);
    EXPECT_GT(received->error_z_m, 0.0);
    EXPECT_EQ(node->get_parameter("gps_topic").as_string(), "/gps/fix");
    EXPECT_EQ(
        node->get_parameter("output_topic").as_string(),
        "/geolocation/position");
}
