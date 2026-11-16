#include <chrono>
#include <cstdio>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <thread>

#include <geolocation_msgs/msg/building_risk.hpp>
#include <geolocation_msgs/msg/geolocation_state.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "geolocation/building_risk_node.hpp"

namespace {

std::string makeBuildingsFile(const std::string& row) {
    const auto timestamp = std::chrono::steady_clock::now().time_since_epoch().count();
    const std::string path =
        "/tmp/geolocation_building_risk_node_" + std::to_string(timestamp) + ".csv";
    std::ofstream output(path);
    output << "id,latitude_deg,longitude_deg,radius_m,base_alt_m,height_m\n" << row << '\n';
    return path;
}

class TemporaryFile {
public:
    explicit TemporaryFile(std::string path) : path_(std::move(path)) {}
    ~TemporaryFile() { std::remove(path_.c_str()); }
    const std::string& path() const { return path_; }

private:
    std::string path_;
};

class BuildingRiskNodeTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        int argc = 0;
        char** argv = nullptr;
        rclcpp::init(argc, argv);
    }

    static void TearDownTestSuite() {
        rclcpp::shutdown();
    }

    static void spinFor(rclcpp::executors::SingleThreadedExecutor& executor) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(150);
        while (std::chrono::steady_clock::now() < deadline) {
            executor.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    static rclcpp::NodeOptions optionsFor(const std::string& path) {
        rclcpp::NodeOptions options;
        options.parameter_overrides({
            rclcpp::Parameter("buildings_file", path),
            rclcpp::Parameter("output_topic", "/test/building_risk")});
        return options;
    }

    static geolocation_msgs::msg::GeolocationState::SharedPtr position() {
        auto message = std::make_shared<geolocation_msgs::msg::GeolocationState>();
        message->header.frame_id = "gps";
        message->header.stamp.sec = 123;
        message->header.stamp.nanosec = 456;
        message->latitude_deg = 34.7;
        message->longitude_deg = -86.6;
        message->altitude_m = 200.0;
        message->error_x_m = 3.0;
        message->error_y_m = 3.0;
        message->error_z_m = 3.0;
        return message;
    }
};

}  // namespace

TEST_F(BuildingRiskNodeTest, PublishesRiskAndCopiesPositionHeader) {
    TemporaryFile file(makeBuildingsFile("center,34.7,-86.6,10,190,30"));
    auto node = std::make_shared<BuildingRiskNode>(optionsFor(file.path()));
    auto observer = std::make_shared<rclcpp::Node>("building_risk_observer");
    geolocation_msgs::msg::BuildingRisk::SharedPtr received;
    auto subscription = observer->create_subscription<geolocation_msgs::msg::BuildingRisk>(
        "/test/building_risk", rclcpp::QoS(10),
        [&received](geolocation_msgs::msg::BuildingRisk::SharedPtr message) {
            received = std::move(message);
        });
    (void)subscription;
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(observer);

    node->onPosition(position());
    spinFor(executor);

    ASSERT_TRUE(received);
    EXPECT_EQ(received->header.frame_id, "gps");
    EXPECT_EQ(received->header.stamp.sec, 123);
    EXPECT_EQ(received->header.stamp.nanosec, 456U);
    EXPECT_DOUBLE_EQ(received->probability, 1.0);
    EXPECT_DOUBLE_EQ(received->building_distance_m, 0.0);
    EXPECT_DOUBLE_EQ(received->building_height_m, 220.0);
}

TEST_F(BuildingRiskNodeTest, PublishesNothingWhenNoBuildingIsInRange) {
    TemporaryFile file(makeBuildingsFile("far,34.8,-86.6,10,190,30"));
    rclcpp::NodeOptions options = optionsFor(file.path());
    options.parameter_overrides({
        rclcpp::Parameter("buildings_file", file.path()),
        rclcpp::Parameter("output_topic", "/test/building_risk_far"),
        rclcpp::Parameter("max_range_m", 50.0)});
    auto node = std::make_shared<BuildingRiskNode>(options);
    auto observer = std::make_shared<rclcpp::Node>("building_risk_far_observer");
    int received = 0;
    auto subscription = observer->create_subscription<geolocation_msgs::msg::BuildingRisk>(
        "/test/building_risk_far", rclcpp::QoS(10),
        [&received](geolocation_msgs::msg::BuildingRisk::SharedPtr) { ++received; });
    (void)subscription;
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(observer);

    node->onPosition(position());
    spinFor(executor);
    EXPECT_EQ(received, 0);
}

TEST_F(BuildingRiskNodeTest, MissingBuildingFileProducesNoOutput) {
    const std::string path = "/tmp/geolocation_missing_buildings_for_node_test.csv";
    auto node = std::make_shared<BuildingRiskNode>(optionsFor(path));
    auto observer = std::make_shared<rclcpp::Node>("building_risk_missing_observer");
    int received = 0;
    auto subscription = observer->create_subscription<geolocation_msgs::msg::BuildingRisk>(
        "/test/building_risk", rclcpp::QoS(10),
        [&received](geolocation_msgs::msg::BuildingRisk::SharedPtr) { ++received; });
    (void)subscription;
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(observer);

    node->onPosition(position());
    spinFor(executor);
    EXPECT_EQ(received, 0);
}

TEST_F(BuildingRiskNodeTest, NonFiniteAndNullPositionsDoNotPublishOrThrow) {
    TemporaryFile file(makeBuildingsFile("center,34.7,-86.6,10,190,30"));
    auto node = std::make_shared<BuildingRiskNode>(optionsFor(file.path()));
    auto observer = std::make_shared<rclcpp::Node>("building_risk_invalid_observer");
    int received = 0;
    auto subscription = observer->create_subscription<geolocation_msgs::msg::BuildingRisk>(
        "/test/building_risk", rclcpp::QoS(10),
        [&received](geolocation_msgs::msg::BuildingRisk::SharedPtr) { ++received; });
    (void)subscription;
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(observer);

    auto invalid = position();
    invalid->latitude_deg = std::numeric_limits<double>::quiet_NaN();
    EXPECT_NO_THROW(node->onPosition(invalid));
    EXPECT_NO_THROW(node->onPosition(nullptr));
    spinFor(executor);
    EXPECT_EQ(received, 0);
}

TEST_F(BuildingRiskNodeTest, DeclaresExpectedDefaultParameters) {
    auto node = std::make_shared<BuildingRiskNode>();
    EXPECT_EQ(
        node->get_parameter("input_topic").as_string(), "/geolocation/position");
    EXPECT_EQ(
        node->get_parameter("output_topic").as_string(), "/geolocation/building_risk");
    EXPECT_NEAR(node->get_parameter("max_range_m").as_double(), 200.0, 1e-12);
    EXPECT_NE(
        node->get_parameter("buildings_file").as_string().find(
            "config/buildings_huntsville.csv"),
        std::string::npos);
}
