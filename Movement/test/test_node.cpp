#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "movement/movement_node.hpp"

using namespace movement;

namespace {

struct Velocity {
    float forward, right, down, yaw;
};

// Everything the node asked the vehicle to do, plus the canned answers it gets back.
struct FakeState {
    std::optional<VehicleState> state;
    Status arm_result;
    Status takeoff_result;
    Status offboard_result;
    Status velocity_result;
    int arms{0};
    int takeoffs{0};
    int offboard_starts{0};
    int offboard_stops{0};
    std::vector<Velocity> velocities;
};

class FakeVehicle : public VehicleInterface {
public:
    explicit FakeVehicle(FakeState* s) : s_(s) {}
    std::optional<VehicleState> state() override { return s_->state; }
    Status arm() override { ++s_->arms; return s_->arm_result; }
    Status takeoff() override { ++s_->takeoffs; return s_->takeoff_result; }
    Status startOffboard() override { ++s_->offboard_starts; return s_->offboard_result; }
    Status setVelocityBody(float f, float r, float d, float y) override {
        s_->velocities.push_back({f, r, d, y});
        return s_->velocity_result;
    }
    Status stopOffboard() override { ++s_->offboard_stops; return {}; }

private:
    FakeState* s_;
};

sensor_msgs::msg::CameraInfo cameraInfo() {
    sensor_msgs::msg::CameraInfo c;
    c.width = 1920;
    c.height = 1080;
    c.k[0] = 1000.0;
    c.k[2] = 960.0;
    c.k[4] = 1000.0;
    return c;
}

// A person whose box is centred on `x_px` and tall enough to be `range_m` away (fy = 1000).
perception_msgs::msg::Detection person(double x_px, double range_m) {
    perception_msgs::msg::Detection d;
    const double height = 1.9 * 1000.0 / range_m;
    d.confidence = 0.9F;
    d.x_min = static_cast<float>(x_px - 50.0);
    d.x_max = static_cast<float>(x_px + 50.0);
    d.y_min = 100.0F;
    d.y_max = static_cast<float>(100.0 + height);
    return d;
}

VehicleState flying() { return {true, true}; }

class NodeTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }

    void SetUp() override { node = make({}); }

    std::shared_ptr<MovementNode> make(const std::vector<rclcpp::Parameter>& overrides,
                                       FakeState* into = nullptr) {
        rclcpp::NodeOptions opts;
        opts.parameter_overrides(overrides);
        FakeState* target = into != nullptr ? into : &fake;
        target->state = flying();
        return std::make_shared<MovementNode>(std::make_unique<FakeVehicle>(target), opts);
    }

    static rclcpp::Time t(double s) {
        return rclcpp::Time(static_cast<int64_t>(s * 1e9), RCL_ROS_TIME);
    }

    FakeState fake;
    std::shared_ptr<MovementNode> node;
};

}  // namespace

// ---------- parameters ----------

TEST_F(NodeTest, ParameterDefaultsAreTheOriginalControllersAndSafe) {
    EXPECT_DOUBLE_EQ(node->get_parameter("safe_distance_m").as_double(), 6.0);
    EXPECT_DOUBLE_EQ(node->get_parameter("min_distance_m").as_double(), 4.0);
    EXPECT_DOUBLE_EQ(node->get_parameter("max_distance_m").as_double(), 9.0);
    EXPECT_DOUBLE_EQ(node->get_parameter("max_forward_m_s").as_double(), 0.7);
    EXPECT_DOUBLE_EQ(node->get_parameter("max_strafe_m_s").as_double(), 0.45);
    EXPECT_DOUBLE_EQ(node->get_parameter("max_yaw_rate_deg_s").as_double(), 45.0);
    EXPECT_EQ(node->get_parameter("target_timeout_ms").as_int(), 800);
    EXPECT_DOUBLE_EQ(node->get_parameter("search_yaw_rate_deg_s").as_double(), 25.0);
    EXPECT_DOUBLE_EQ(node->get_parameter("search_forward_m_s").as_double(), 0.35);
    EXPECT_DOUBLE_EQ(node->get_parameter("search_strafe_m_s").as_double(), 0.45);
    EXPECT_DOUBLE_EQ(node->get_parameter("search_orbit_hz").as_double(), 0.25);
    EXPECT_EQ(node->get_parameter("detection_topic").as_string(), "/perception/detection");
    EXPECT_EQ(node->get_parameter("camera_info_topic").as_string(),
              "/camera/front/camera_info");
    EXPECT_DOUBLE_EQ(node->get_parameter("person_height_m").as_double(), 1.9);
    EXPECT_FALSE(node->get_parameter("auto_arm_takeoff").as_bool());
    EXPECT_DOUBLE_EQ(node->get_parameter("search_timeout_s").as_double(), 0.0);
    EXPECT_DOUBLE_EQ(node->get_parameter("forward_gain").as_double(), 0.35);
    EXPECT_DOUBLE_EQ(node->get_parameter("lateral_gain").as_double(), 0.7);
    EXPECT_DOUBLE_EQ(node->get_parameter("yaw_gain").as_double(), 1.0);
    EXPECT_DOUBLE_EQ(node->get_parameter("orbit_yaw_ki").as_double(), 30.0);
    EXPECT_DOUBLE_EQ(node->get_parameter("max_forward_accel_m_s2").as_double(), 0.0);
    EXPECT_DOUBLE_EQ(node->get_parameter("max_strafe_accel_m_s2").as_double(), 0.0);
    EXPECT_DOUBLE_EQ(node->get_parameter("max_yaw_accel_deg_s2").as_double(), 0.0);
    EXPECT_DOUBLE_EQ(node->get_parameter("last_known_search_s").as_double(), 0.0);
    EXPECT_DOUBLE_EQ(node->get_parameter("last_known_sweep_deg").as_double(), 60.0);
    EXPECT_DOUBLE_EQ(node->get_parameter("last_known_rate_deg_s").as_double(), 20.0);
    EXPECT_FALSE(node->get_parameter("orbit_enabled").as_bool());
    EXPECT_FALSE(node->get_parameter("orbit_clockwise").as_bool());
    EXPECT_DOUBLE_EQ(node->get_parameter("orbit_period_s").as_double(), 30.0);
    EXPECT_DOUBLE_EQ(node->get_parameter("max_orbit_speed_m_s").as_double(), 1.5);
}

// ---------- arming ----------

TEST_F(NodeTest, NeverArmsOrTakesOffByDefault) {
    fake.state = VehicleState{false, false};
    node->tick(t(1.0));
    fake.state = VehicleState{true, false};
    node->tick(t(1.1));
    EXPECT_EQ(fake.arms, 0);
    EXPECT_EQ(fake.takeoffs, 0);
    EXPECT_EQ(fake.offboard_starts, 0);
    EXPECT_TRUE(fake.velocities.empty());
}

TEST_F(NodeTest, ArmsAndTakesOffWhenAskedTo) {
    FakeState f;
    auto n = make({{"auto_arm_takeoff", true}}, &f);
    f.state = VehicleState{false, false};
    n->tick(t(1.0));
    EXPECT_EQ(f.arms, 1);
    EXPECT_EQ(f.takeoffs, 1);
    f.state = VehicleState{true, false};  // already armed: only take off
    n->tick(t(1.1));
    EXPECT_EQ(f.arms, 1);
    EXPECT_EQ(f.takeoffs, 2);
    EXPECT_TRUE(f.velocities.empty());  // no offboard motion until airborne
}

TEST_F(NodeTest, FailedArmStopsBeforeTakeoffAndDoesNotCrash) {
    FakeState f;
    auto n = make({{"auto_arm_takeoff", true}}, &f);
    f.state = VehicleState{false, false};
    f.arm_result = {false, "denied"};
    EXPECT_NO_THROW(n->tick(t(1.0)));
    EXPECT_EQ(f.takeoffs, 0);
}

// ---------- offboard lifecycle ----------

TEST_F(NodeTest, StartsOffboardOnceOnceAirborne) {
    node->tick(t(1.0));
    node->tick(t(1.1));
    node->tick(t(1.2));
    EXPECT_EQ(fake.offboard_starts, 1);
    EXPECT_TRUE(node->offboardActive());
}

TEST_F(NodeTest, FailedOffboardStartIsRetriedAndSendsNothing) {
    fake.offboard_result = {false, "no"};
    node->tick(t(1.0));
    node->tick(t(1.1));
    EXPECT_EQ(fake.offboard_starts, 2);
    EXPECT_FALSE(node->offboardActive());
    EXPECT_TRUE(fake.velocities.empty());
}

TEST_F(NodeTest, OffboardRestartsAfterALandingAndNewTakeoff) {
    // The original set its started-flag once and never cleared it, so a second flight
    // never re-entered offboard.
    node->tick(t(1.0));
    ASSERT_EQ(fake.offboard_starts, 1);
    fake.state = VehicleState{false, false};
    node->tick(t(2.0));
    EXPECT_FALSE(node->offboardActive());
    fake.state = flying();
    node->tick(t(3.0));
    EXPECT_EQ(fake.offboard_starts, 2);
}

TEST_F(NodeTest, LostConnectionSendsNothingAndRestartsOffboardOnReconnect) {
    node->tick(t(1.0));
    const size_t sent = fake.velocities.size();
    fake.state.reset();
    node->tick(t(1.1));
    node->tick(t(1.2));
    EXPECT_EQ(fake.velocities.size(), sent);
    EXPECT_FALSE(node->offboardActive());
    fake.state = flying();
    node->tick(t(2.0));
    EXPECT_EQ(fake.offboard_starts, 2);
}

TEST_F(NodeTest, ShutdownLeavesOffboardOnlyWhenItWasStartedAndOnlyOnce) {
    node->shutdown();
    EXPECT_EQ(fake.offboard_stops, 0);
    node->tick(t(1.0));
    node->shutdown();
    node->shutdown();
    EXPECT_EQ(fake.offboard_stops, 1);
}

TEST_F(NodeTest, DestructorAlsoHandsBackToHold) {
    FakeState f;
    {
        auto n = make({}, &f);
        n->tick(t(1.0));
    }
    EXPECT_EQ(f.offboard_stops, 1);
}

// ---------- following ----------

TEST_F(NodeTest, CentredPersonAtSafeDistanceCommandsNoMotion) {
    node->onCameraInfo(cameraInfo());
    node->onDetection(person(960.0, 6.0), t(1.0));
    node->tick(t(1.1));
    ASSERT_EQ(fake.velocities.size(), 1U);
    EXPECT_NEAR(fake.velocities[0].forward, 0.0F, 1e-4);
    EXPECT_NEAR(fake.velocities[0].right, 0.0F, 1e-6);
    EXPECT_FLOAT_EQ(fake.velocities[0].down, 0.0F);  // altitude is never commanded
    EXPECT_NEAR(fake.velocities[0].yaw, 0.0F, 1e-6);
}

TEST_F(NodeTest, OffCentreAndTooFarMatchesTheOriginalFormulas) {
    node->onCameraInfo(cameraInfo());
    node->onDetection(person(1440.0, 7.0), t(1.0));  // half right, 1 m too far
    node->tick(t(1.1));
    ASSERT_EQ(fake.velocities.size(), 1U);
    EXPECT_NEAR(fake.velocities[0].forward, 0.35F, 1e-4);
    EXPECT_NEAR(fake.velocities[0].right, 0.1575F, 1e-5);
    EXPECT_NEAR(fake.velocities[0].yaw, 22.5F, 1e-4);
}

TEST_F(NodeTest, PersonHeightParameterChangesTheRange) {
    FakeState f;
    auto n = make({{"person_height_m", 1.0}}, &f);
    n->onCameraInfo(cameraInfo());
    n->onDetection(person(960.0, 6.0), t(1.0));  // box sized for 1.9 m, so 1.0 m reads closer
    n->tick(t(1.1));
    ASSERT_EQ(f.velocities.size(), 1U);
    EXPECT_LT(f.velocities[0].forward, 0.0F);
}

TEST_F(NodeTest, TargetAtExactlyTheTimeoutIsStillFollowed) {
    node->onCameraInfo(cameraInfo());
    node->onDetection(person(960.0, 6.0), t(1.0));
    node->tick(t(1.8));  // 800 ms
    EXPECT_FALSE(node->searching());
}

TEST_F(NodeTest, TargetInViewWithoutCameraInfoHoldsPosition) {
    // The original scanned here although the person was in view.
    node->onDetection(person(1440.0, 9.0), t(1.0));
    node->tick(t(1.1));
    ASSERT_EQ(fake.velocities.size(), 1U);
    EXPECT_FLOAT_EQ(fake.velocities[0].forward, 0.0F);
    EXPECT_FLOAT_EQ(fake.velocities[0].right, 0.0F);
    EXPECT_FLOAT_EQ(fake.velocities[0].yaw, 0.0F);
    EXPECT_FALSE(node->searching());
}

TEST_F(NodeTest, CameraInfoArrivingLaterIsUsedOnTheNextTick) {
    node->onDetection(person(1440.0, 7.0), t(1.0));
    node->tick(t(1.1));
    node->onCameraInfo(cameraInfo());
    node->onDetection(person(1440.0, 7.0), t(1.2));
    node->tick(t(1.3));
    EXPECT_NEAR(fake.velocities.back().forward, 0.35F, 1e-4);
}

TEST_F(NodeTest, UnusableCameraInfoIsNotUsed) {
    auto c = cameraInfo();
    c.k[0] = 0.0;
    node->onCameraInfo(c);
    node->onDetection(person(1440.0, 7.0), t(1.0));
    node->tick(t(1.1));
    EXPECT_FLOAT_EQ(fake.velocities.back().yaw, 0.0F);
}

TEST_F(NodeTest, BrokenDetectionIsIgnored) {
    node->onCameraInfo(cameraInfo());
    auto d = person(960.0, 6.0);
    d.x_min = std::nanf("");
    node->onDetection(d, t(1.0));
    node->tick(t(1.1));
    EXPECT_TRUE(node->searching());  // never saw a valid target
}

TEST_F(NodeTest, FailedVelocityCommandDoesNotCrash) {
    fake.velocity_result = {false, "link"};
    node->onCameraInfo(cameraInfo());
    node->onDetection(person(960.0, 6.0), t(1.0));
    EXPECT_NO_THROW(node->tick(t(1.1)));
}

TEST_F(NodeTest, ParametersCannotExceedHardCaps) {
    FakeState f;
    auto n = make({{"max_forward_m_s", 99.0}, {"max_strafe_m_s", 99.0},
                   {"max_yaw_rate_deg_s", 9999.0}},
                  &f);
    n->onCameraInfo(cameraInfo());
    n->onDetection(person(1920.0, 9.0), t(1.0));
    n->tick(t(1.1));
    ASSERT_EQ(f.velocities.size(), 1U);
    EXPECT_LE(f.velocities[0].forward, kForwardSpeedCeilingMps + 1e-6);
    EXPECT_LE(f.velocities[0].right, kStrafeSpeedCeilingMps + 1e-6);
    EXPECT_LE(f.velocities[0].yaw, kYawRateCeilingDegS + 1e-6);
}

// ---------- boxes cut off by the frame edge ----------

TEST_F(NodeTest, ClippedBoxGivesNoRangeSoNoForwardMotion) {
    node->onCameraInfo(cameraInfo());  // 1080 px tall
    auto d = person(960.0, 6.0);
    d.y_min = 800.0F;
    d.y_max = 1079.0F;  // feet below the frame: its height, and so its range, cannot be trusted
    node->onDetection(d, t(1.0));
    node->tick(t(1.1));
    ASSERT_EQ(fake.velocities.size(), 1U);
    EXPECT_FLOAT_EQ(fake.velocities[0].forward, 0.0F);
}

TEST_F(NodeTest, UncutBoxStillGivesARange) {
    node->onCameraInfo(cameraInfo());
    node->onDetection(person(960.0, 7.0), t(1.0));
    node->tick(t(1.1));
    ASSERT_EQ(fake.velocities.size(), 1U);
    EXPECT_NEAR(fake.velocities[0].forward, 0.35F, 1e-4);
}

// ---------- smoothing ----------

TEST_F(NodeTest, AccelerationLimitsSmoothTheCommandsAndRestartFromRest) {
    FakeState f;
    auto n = make({{"max_forward_accel_m_s2", 1.0}, {"max_yaw_accel_deg_s2", 100.0}}, &f);
    n->onCameraInfo(cameraInfo());
    for (int i = 0; i < 3; ++i) {
        n->onDetection(person(1920.0, 9.0), t(1.0 + i * 0.1));  // far, at the right edge
        n->tick(t(1.05 + i * 0.1));
    }
    ASSERT_EQ(f.velocities.size(), 3U);
    EXPECT_NEAR(f.velocities[0].forward, 0.1F, 1e-5);  // 1 m/s^2 * 0.1 s, not the full 0.7
    EXPECT_NEAR(f.velocities[1].forward, 0.2F, 1e-5);
    EXPECT_NEAR(f.velocities[2].forward, 0.3F, 1e-5);
    EXPECT_NEAR(f.velocities[0].yaw, 10.0F, 1e-4);
}

TEST_F(NodeTest, TunedGainsAreUsed) {
    FakeState f;
    auto n = make({{"forward_gain", 0.7}, {"yaw_gain", 0.5}}, &f);
    n->onCameraInfo(cameraInfo());
    n->onDetection(person(1440.0, 7.0), t(1.0));  // half right, 1 m too far
    n->tick(t(1.1));
    ASSERT_EQ(f.velocities.size(), 1U);
    EXPECT_NEAR(f.velocities[0].forward, 0.7F, 1e-4);
    EXPECT_NEAR(f.velocities[0].yaw, 11.25F, 1e-4);
}

// ---------- orbit ----------

TEST_F(NodeTest, FollowsRatherThanOrbitsByDefault) {
    node->onCameraInfo(cameraInfo());
    node->onDetection(person(960.0, 6.0), t(1.0));
    node->tick(t(1.1));
    ASSERT_EQ(fake.velocities.size(), 1U);
    EXPECT_NEAR(fake.velocities[0].right, 0.0F, 1e-6);  // no sideways motion when centred
}

TEST_F(NodeTest, OrbitModeCirclesTheCentredPerson) {
    FakeState f;
    auto n = make({{"orbit_enabled", true}}, &f);
    n->onCameraInfo(cameraInfo());
    n->onDetection(person(960.0, 6.0), t(1.0));
    n->tick(t(1.1));
    ASSERT_EQ(f.velocities.size(), 1U);
    EXPECT_NEAR(f.velocities[0].right, 2.0 * kPi * 6.0 / 30.0, 1e-4);
    EXPECT_NEAR(f.velocities[0].yaw, 0.0F, 1e-4);  // centred: no open-loop turn command
    EXPECT_FLOAT_EQ(f.velocities[0].down, 0.0F);
}

TEST_F(NodeTest, OrbitDirectionAndPeriodAreParameters) {
    FakeState f;
    auto n = make({{"orbit_enabled", true}, {"orbit_clockwise", true}, {"orbit_period_s", 60.0}},
                  &f);
    n->onCameraInfo(cameraInfo());
    n->onDetection(person(960.0, 6.0), t(1.0));
    n->tick(t(1.1));
    ASSERT_EQ(f.velocities.size(), 1U);
    EXPECT_NEAR(f.velocities[0].right, -2.0 * kPi * 6.0 / 60.0, 1e-4);
    EXPECT_NEAR(f.velocities[0].yaw, 0.0F, 1e-4);
}

TEST_F(NodeTest, OrbitSpeedCannotExceedItsHardCap) {
    FakeState f;
    auto n = make({{"orbit_enabled", true}, {"orbit_period_s", 5.0},
                   {"max_orbit_speed_m_s", 99.0}, {"safe_distance_m", 9.0}},
                  &f);
    n->onCameraInfo(cameraInfo());
    n->onDetection(person(960.0, 9.0), t(1.0));
    n->tick(t(1.1));
    ASSERT_EQ(f.velocities.size(), 1U);
    EXPECT_LE(std::abs(f.velocities[0].right), kOrbitSpeedCeilingMps + 1e-6);
}

TEST_F(NodeTest, LosingTheTargetWhileOrbitingFallsBackToTheScan) {
    FakeState f;
    auto n = make({{"orbit_enabled", true}}, &f);
    n->onCameraInfo(cameraInfo());
    n->onDetection(person(960.0, 6.0), t(1.0));
    n->tick(t(1.1));
    n->tick(t(3.0));  // detection is now older than the timeout
    EXPECT_TRUE(n->searching());
    ASSERT_EQ(f.velocities.size(), 2U);
    EXPECT_NEAR(f.velocities[1].yaw, 25.0F, 1e-4);  // the scan's constant yaw, not the orbit's
}

TEST_F(NodeTest, OrbitYawIntegralIsForgottenWhenTheTargetIsLost) {
    FakeState f;
    auto n = make({{"orbit_enabled", true}}, &f);
    n->onCameraInfo(cameraInfo());
    for (int i = 0; i < 10; ++i) {  // person stuck right of centre: the integral builds up
        n->onDetection(person(1440.0, 6.0), t(1.0 + i * 0.1));
        n->tick(t(1.05 + i * 0.1));
    }
    n->tick(t(5.0));  // target lost, scan runs
    n->onDetection(person(1440.0, 6.0), t(6.0));
    n->tick(t(6.05));
    const float first_after = f.velocities.back().yaw;
    f.velocities.clear();
    FakeState g;
    auto fresh = make({{"orbit_enabled", true}}, &g);
    fresh->onCameraInfo(cameraInfo());
    fresh->onDetection(person(1440.0, 6.0), t(1.0));
    fresh->tick(t(1.05));
    EXPECT_NEAR(first_after, g.velocities.back().yaw, 1e-4);  // same as a brand-new controller
}

// ---------- target lost: search around the last known direction ----------

TEST_F(NodeTest, LostTargetSweepsInPlaceTowardTheSideItWasLastSeen) {
    FakeState f;
    auto n = make({{"last_known_search_s", 8.0}}, &f);
    n->onCameraInfo(cameraInfo());
    n->onDetection(person(1500.0, 6.0), t(1.0));  // right of centre
    n->tick(t(1.05));
    f.velocities.clear();
    n->tick(t(2.5));  // detection older than the 800 ms timeout
    n->tick(t(3.5));
    ASSERT_GE(f.velocities.size(), 2U);
    for (const auto& v : f.velocities) {
        EXPECT_FLOAT_EQ(v.forward, 0.0F);   // holds position: no creeping, no sideways sweep
        EXPECT_FLOAT_EQ(v.right, 0.0F);
        EXPECT_GT(v.yaw, 0.0F);             // turns right, toward where it was last seen
    }
}

TEST_F(NodeTest, LostTargetOnTheLeftSweepsLeftFirst) {
    FakeState f;
    auto n = make({{"last_known_search_s", 8.0}}, &f);
    n->onCameraInfo(cameraInfo());
    n->onDetection(person(400.0, 6.0), t(1.0));
    n->tick(t(1.05));
    f.velocities.clear();
    n->tick(t(2.5));
    ASSERT_EQ(f.velocities.size(), 1U);
    EXPECT_LT(f.velocities[0].yaw, 0.0F);
}

TEST_F(NodeTest, AfterTheLastKnownWindowTheSpiralScanTakesOver) {
    FakeState f;
    auto n = make({{"last_known_search_s", 2.0}}, &f);
    n->onCameraInfo(cameraInfo());
    n->onDetection(person(960.0, 6.0), t(1.0));
    n->tick(t(1.05));
    f.velocities.clear();
    n->tick(t(2.5));   // search starts: inside the 2 s window
    n->tick(t(5.0));   // 2.5 s after the start: past the window
    ASSERT_EQ(f.velocities.size(), 2U);
    EXPECT_FLOAT_EQ(f.velocities[0].forward, 0.0F);   // sweep: no translation
    EXPECT_NEAR(f.velocities[1].forward, 0.28F, 1e-4);  // original spiral: creeps forward
    EXPECT_NEAR(f.velocities[1].yaw, 25.0F, 1e-4);
    EXPECT_TRUE(n->searching());
}

TEST_F(NodeTest, WithoutTheOptionTheOriginalSpiralRunsFromTheStart) {
    node->onCameraInfo(cameraInfo());
    node->onDetection(person(1500.0, 6.0), t(1.0));
    node->tick(t(1.05));
    fake.velocities.clear();
    node->tick(t(2.5));
    ASSERT_EQ(fake.velocities.size(), 1U);
    EXPECT_NEAR(fake.velocities[0].forward, 0.28F, 1e-4);
    EXPECT_NEAR(fake.velocities[0].yaw, 25.0F, 1e-4);
}

TEST_F(NodeTest, ReacquiringTheTargetEndsTheSweepAndFollowsAgain) {
    FakeState f;
    auto n = make({{"last_known_search_s", 8.0}}, &f);
    n->onCameraInfo(cameraInfo());
    n->onDetection(person(960.0, 6.0), t(1.0));
    n->tick(t(1.05));
    n->tick(t(2.5));
    EXPECT_TRUE(n->searching());
    n->onDetection(person(960.0, 7.0), t(3.0));
    n->tick(t(3.05));
    EXPECT_FALSE(n->searching());
    EXPECT_GT(f.velocities.back().forward, 0.0F);   // back to following (1 m too far)
}

// ---------- target lost: search ----------

TEST_F(NodeTest, NoTargetEverMeansTheSpiralScan) {
    node->tick(t(1.0));
    ASSERT_EQ(fake.velocities.size(), 1U);
    EXPECT_TRUE(node->searching());
    EXPECT_NEAR(fake.velocities[0].forward, 0.28F, 1e-6);
    EXPECT_NEAR(fake.velocities[0].right, 0.0F, 1e-6);
    EXPECT_NEAR(fake.velocities[0].yaw, 25.0F, 1e-6);
}

TEST_F(NodeTest, StaleTargetStartsTheScanAndItsSineRunsFromThatMoment) {
    node->onCameraInfo(cameraInfo());
    node->onDetection(person(960.0, 6.0), t(1.0));
    node->tick(t(1.1));
    node->tick(t(2.0));          // 1.0 s old: stale, scan begins here
    EXPECT_TRUE(node->searching());
    EXPECT_NEAR(fake.velocities.back().right, 0.0F, 1e-6);
    node->tick(t(3.0));          // 1 s into the scan = quarter period at 0.25 Hz
    EXPECT_NEAR(fake.velocities.back().right, 0.45F, 1e-5);
}

TEST_F(NodeTest, TargetReturningStopsTheScanAndRestartsItFromZeroNextTime) {
    node->onCameraInfo(cameraInfo());
    node->tick(t(1.0));
    node->tick(t(2.0));
    ASSERT_TRUE(node->searching());
    node->onDetection(person(960.0, 6.0), t(3.0));
    node->tick(t(3.1));
    EXPECT_FALSE(node->searching());
    node->tick(t(10.0));  // lost again: new scan starts at phase 0
    EXPECT_TRUE(node->searching());
    EXPECT_NEAR(fake.velocities.back().right, 0.0F, 1e-6);
}

TEST_F(NodeTest, ByDefaultTheScanNeverGivesUp) {
    node->tick(t(1.0));
    node->tick(t(5000.0));
    EXPECT_TRUE(node->searching());
    EXPECT_EQ(fake.offboard_stops, 0);
    EXPECT_EQ(fake.velocities.size(), 2U);
}

TEST_F(NodeTest, OptInSearchTimeoutHandsBackToHoldAndWaitsForTheTarget) {
    FakeState f;
    auto n = make({{"search_timeout_s", 5.0}}, &f);
    n->onCameraInfo(cameraInfo());
    n->tick(t(1.0));
    n->tick(t(5.0));
    EXPECT_TRUE(n->searching());
    const size_t sent = f.velocities.size();
    n->tick(t(6.5));  // 5.5 s into the scan
    EXPECT_EQ(f.offboard_stops, 1);
    EXPECT_FALSE(n->offboardActive());
    n->tick(t(8.0));
    n->tick(t(9.0));
    EXPECT_EQ(f.velocities.size(), sent);     // stays in Hold
    EXPECT_EQ(f.offboard_starts, 1);           // and does not flap back into offboard
    n->onDetection(person(960.0, 6.0), t(10.0));
    n->tick(t(10.1));
    EXPECT_EQ(f.offboard_starts, 2);           // target seen: follow again
    EXPECT_GT(f.velocities.size(), sent);
}
