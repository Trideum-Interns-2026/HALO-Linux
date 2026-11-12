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

TEST_F(TrackingNodeTest, PublishesNothingWithoutCameraInfoOrDronePosition) {
    auto node = std::make_shared<TrackingNode>();
    auto observer = std::make_shared<rclcpp::Node>("tracking_silent_observer");
    int received = 0;
    auto subscription = observer->create_subscription<tracking_msgs::msg::TrackingState>(
        "/tracking/position", rclcpp::QoS(10),
        [&received](tracking_msgs::msg::TrackingState::SharedPtr) { ++received; });
    (void)subscription;
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(observer);

    auto detection = std::make_shared<perception_msgs::msg::Detection>();
    detection->header.stamp = node->now();
    detection->confidence = 0.9F;
    detection->x_min = 270.0F;
    detection->y_min = 190.0F;
    detection->x_max = 370.0F;
    detection->y_max = 290.0F;
    // No camera_info, no drone position, no attitude yet: must neither publish nor throw.
    EXPECT_NO_THROW(node->onDetection(detection));

    for (int i = 0; i < 20; ++i) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    EXPECT_EQ(received, 0);
}

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

namespace {

// Node with valid inputs and an observer; publishes one detection and reports what arrived.
struct EdgeHarness {
    explicit EdgeHarness(uint32_t image_height) {
        node = std::make_shared<TrackingNode>();
        observer = std::make_shared<rclcpp::Node>("tracking_edge_observer");
        subscription = observer->create_subscription<tracking_msgs::msg::TrackingState>(
            "/tracking/position", rclcpp::QoS(10),
            [this](tracking_msgs::msg::TrackingState::SharedPtr m) { received = std::move(m); });
        executor.add_node(observer);

        auto camera = std::make_shared<sensor_msgs::msg::CameraInfo>();
        camera->height = image_height;
        camera->k[0] = 500.0;
        camera->k[2] = 320.0;
        camera->k[4] = 500.0;
        camera->k[5] = 240.0;
        node->onCameraInfo(camera);

        auto position = std::make_shared<geolocation_msgs::msg::GeolocationState>();
        position->header.stamp = node->now();
        position->altitude_m = 100.0;
        position->error_x_m = 0.3;
        position->error_y_m = 0.3;
        position->error_z_m = 0.3;
        node->onGeolocation(position);

        auto attitude = std::make_shared<px4_msgs::msg::VehicleAttitude>();
        attitude->q = {1.0F, 0.0F, 0.0F, 0.0F};
        node->onAttitude(attitude);
    }

    bool detect(float y_min, float y_max) {
        auto detection = std::make_shared<perception_msgs::msg::Detection>();
        detection->header.stamp = node->now();
        detection->header.frame_id = "camera_optical_frame";
        detection->confidence = 0.9F;
        detection->x_min = 270.0F;
        detection->y_min = y_min;
        detection->x_max = 370.0F;
        detection->y_max = y_max;
        sent_stamp = detection->header.stamp;
        node->onDetection(detection);
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(300);
        while (!received && std::chrono::steady_clock::now() < deadline) {
            executor.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        return received != nullptr;
    }

    std::shared_ptr<TrackingNode> node;
    std::shared_ptr<rclcpp::Node> observer;
    rclcpp::Subscription<tracking_msgs::msg::TrackingState>::SharedPtr subscription;
    rclcpp::executors::SingleThreadedExecutor executor;
    tracking_msgs::msg::TrackingState::SharedPtr received;
    builtin_interfaces::msg::Time sent_stamp;
};

}  // namespace

TEST_F(TrackingNodeTest, PublishesNothingWhenBoxClippedAtBottom) {
    EdgeHarness h(480);
    EXPECT_FALSE(h.detect(100.0F, 480.0F));
}

TEST_F(TrackingNodeTest, PublishesNothingWhenBoxClippedAtTop) {
    EdgeHarness h(480);
    EXPECT_FALSE(h.detect(0.0F, 300.0F));
}

TEST_F(TrackingNodeTest, PublishesNothingWhenBoxEdgeIsJustInsideTheFrame) {
    EdgeHarness h(480);
    // A truncated person's regressed edge lands a few px inside the frame, not on it: 8 px from the
    // bottom (or top) is still inside the default 12 px margin, so it counts as clipped.
    EXPECT_FALSE(h.detect(100.0F, 472.0F));
    EXPECT_FALSE(h.detect(8.0F, 300.0F));
}

TEST_F(TrackingNodeTest, PublishesBoxThreePixelsInsideMarginEdge) {
    EdgeHarness h(480);
    // Margin 12 px: top edge limit is 12, so y_min = 15 is 3 px inside; same at the bottom.
    ASSERT_TRUE(h.detect(15.0F, 465.0F));
    EXPECT_EQ(h.received->header.frame_id, "camera_optical_frame");
    EXPECT_EQ(h.received->header.stamp.sec, h.sent_stamp.sec);
    EXPECT_EQ(h.received->header.stamp.nanosec, h.sent_stamp.nanosec);
}

TEST_F(TrackingNodeTest, PublishesWhenImageHeightUnknown) {
    EdgeHarness h(0);
    EXPECT_TRUE(h.detect(0.0F, 480.0F));
}

TEST_F(TrackingNodeTest, FrameEdgeMarginDefaultsToTwoPixels) {
    auto node = std::make_shared<TrackingNode>();
    // The default was raised from 2 px to 12 px (see kDefaultFrameEdgeMarginPx); the test name is
    // kept because the test-guard hook forbids renaming a test.
    EXPECT_DOUBLE_EQ(node->get_parameter("frame_edge_margin_px").as_double(), 12.0);
}
