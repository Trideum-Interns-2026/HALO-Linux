#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
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
    std::optional<double> heading_deg{0.0};  // what the fake reports as the drone heading
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
    std::optional<VehicleState> state() override {
        if (!s_->state) {
            return std::nullopt;
        }
        VehicleState v = *s_->state;
        v.heading_deg = s_->heading_deg;
        return v;
    }
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

// The drone hovers at (kDroneLat, kDroneLon, kDroneAlt). Positions are built from metres north and
// east with the same ellipsoid radii the node uses, so the expected numbers match the old ones.
constexpr double kDroneLat = 34.7;
constexpr double kDroneLon = -86.6;
constexpr double kDroneAlt = 10.0;
constexpr double kDegPerRad = 180.0 / kPi;

double meridianRadius(double lat_deg) {
    const double s = std::sin(lat_deg / kDegPerRad);
    const double w = 1.0 - kWgs84EccSquared * s * s;
    return kWgs84SemiMajorM * (1.0 - kWgs84EccSquared) / (w * std::sqrt(w));
}

double primeVerticalRadius(double lat_deg) {
    const double s = std::sin(lat_deg / kDegPerRad);
    return kWgs84SemiMajorM / std::sqrt(1.0 - kWgs84EccSquared * s * s);
}

tracking_msgs::msg::TrackingState trackingMsg(double north_m, double east_m,
                                              double altitude_m = kDroneAlt) {
    tracking_msgs::msg::TrackingState m;
    m.latitude_deg = kDroneLat + north_m / meridianRadius(kDroneLat) * kDegPerRad;
    m.longitude_deg = kDroneLon + east_m /
                                      (primeVerticalRadius(kDroneLat) *
                                       std::cos(kDroneLat / kDegPerRad)) * kDegPerRad;
    m.altitude_m = altitude_m;
    return m;
}

geolocation_msgs::msg::GeolocationState droneMsg(double altitude_m = kDroneAlt) {
    geolocation_msgs::msg::GeolocationState m;
    m.latitude_deg = kDroneLat;
    m.longitude_deg = kDroneLon;
    m.altitude_m = altitude_m;
    return m;
}

// A person `range_m` away (horizontally) at `rel_bearing_deg` from the nose of a drone that heads
// `heading_deg` (+ = to the right). Feeds the tracking and the geolocation message, both received
// at `at`. With heading 0 (the fake's default) this puts the person at pixel
// 960 + 1000 * tan(rel_bearing_deg).
void personAt(MovementNode& n, double rel_bearing_deg, double range_m, const rclcpp::Time& at,
              double heading_deg = 0.0) {
    const double world = (heading_deg + rel_bearing_deg) / kDegPerRad;
    n.onTracking(trackingMsg(range_m * std::cos(world), range_m * std::sin(world)), at);
    n.onGeolocation(droneMsg(), at);
}

// The same, for a person that should appear in image column `x_px` (fx = 1000, cx = 960). Columns
// at the very edge are pulled 0.1 px inside so rounding cannot push them out of view.
void personAtPixel(MovementNode& n, double x_px, double range_m, const rclcpp::Time& at) {
    const double x = std::min(std::max(x_px, 0.1), 1919.9);
    personAt(n, std::atan((x - 960.0) / 1000.0) * kDegPerRad, range_m, at);
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
    EXPECT_EQ(node->get_parameter("camera_info_topic").as_string(),
              "/camera/front/camera_info");
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

TEST_F(NodeTest, ParameterDefaultsOfTheTrackedInputAreSafeAndTheOldOnesAreGone) {
    EXPECT_EQ(node->get_parameter("tracking_topic").as_string(), "/tracking/position");
    EXPECT_EQ(node->get_parameter("geolocation_topic").as_string(), "/geolocation/position");
    EXPECT_DOUBLE_EQ(node->get_parameter("max_target_error_m").as_double(), 0.0);  // gate off
    EXPECT_FALSE(node->has_parameter("detection_topic"));
    EXPECT_FALSE(node->has_parameter("person_height_m"));
}

TEST_F(NodeTest, TheErrorGateLimitIsClampedAndNonFiniteMeansOff) {
    FakeState f;
    auto huge = make({{"max_target_error_m", 1e9}}, &f);
    // Clamped to 1000 m: an error norm of 2000 m is rejected, 500 m accepted.
    huge->onCameraInfo(cameraInfo());
    auto m = trackingMsg(6.0, 0.0);
    m.error_x_m = 2000.0;
    huge->onTracking(m, t(1.0));
    huge->onGeolocation(droneMsg(), t(1.0));
    huge->tick(t(1.1));
    EXPECT_TRUE(huge->searching());
    m.error_x_m = 500.0;
    huge->onTracking(m, t(1.2));
    huge->tick(t(1.3));
    EXPECT_FALSE(huge->searching());

    FakeState g;
    auto nan = make({{"max_target_error_m", std::nan("")}}, &g);  // non-finite: gate off
    nan->onCameraInfo(cameraInfo());
    m.error_x_m = 1e6;
    nan->onTracking(m, t(1.0));
    nan->onGeolocation(droneMsg(), t(1.0));
    nan->tick(t(1.1));
    EXPECT_FALSE(nan->searching());
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
    personAtPixel(*node, 960.0, 6.0, t(1.0));
    node->tick(t(1.1));
    ASSERT_EQ(fake.velocities.size(), 1U);
    EXPECT_NEAR(fake.velocities[0].forward, 0.0F, 1e-4);
    EXPECT_NEAR(fake.velocities[0].right, 0.0F, 1e-6);
    EXPECT_FLOAT_EQ(fake.velocities[0].down, 0.0F);  // altitude is never commanded
    EXPECT_NEAR(fake.velocities[0].yaw, 0.0F, 1e-6);
}

TEST_F(NodeTest, OffCentreAndTooFarMatchesTheOriginalFormulas) {
    node->onCameraInfo(cameraInfo());
    personAtPixel(*node, 1440.0, 7.0, t(1.0));  // half right, 1 m too far
    node->tick(t(1.1));
    ASSERT_EQ(fake.velocities.size(), 1U);
    EXPECT_NEAR(fake.velocities[0].forward, 0.35F, 1e-4);
    EXPECT_NEAR(fake.velocities[0].right, 0.1575F, 1e-5);
    EXPECT_NEAR(fake.velocities[0].yaw, 22.5F, 1e-4);
}

TEST_F(NodeTest, RangeIsTheHorizontalDistanceToTheTrackedPerson) {
    // Replaces the person-height parameter test: the range comes from the two positions, not from
    // a box. 7 m away horizontally is 1 m too far whatever the height difference.
    for (const double person_alt : {10.0, 0.0, 60.0}) {
        FakeState f;
        auto n = make({}, &f);
        n->onCameraInfo(cameraInfo());
        n->onTracking(trackingMsg(7.0, 0.0, person_alt), t(1.0));
        n->onGeolocation(droneMsg(10.0), t(1.0));
        n->tick(t(1.1));
        ASSERT_EQ(f.velocities.size(), 1U) << "person altitude " << person_alt;
        EXPECT_NEAR(f.velocities[0].forward, 0.35F, 1e-4) << "person altitude " << person_alt;
    }
}

TEST_F(NodeTest, TargetAtExactlyTheTimeoutIsStillFollowed) {
    node->onCameraInfo(cameraInfo());
    personAtPixel(*node, 960.0, 6.0, t(1.0));
    node->tick(t(1.8));  // 800 ms
    EXPECT_FALSE(node->searching());
}

TEST_F(NodeTest, TargetWithoutCameraInfoCountsAsNotFreshAndTheScanRuns) {
    // Replaces the old "holding position without camera_info" branch, which is gone: without a
    // usable camera there is no target, so the existing lost-target path (the scan) runs.
    personAtPixel(*node, 1440.0, 9.0, t(1.0));
    node->tick(t(1.1));
    ASSERT_EQ(fake.velocities.size(), 1U);
    EXPECT_TRUE(node->searching());
    EXPECT_NEAR(fake.velocities[0].forward, 0.28F, 1e-6);
    EXPECT_NEAR(fake.velocities[0].yaw, 25.0F, 1e-6);
}

TEST_F(NodeTest, CameraInfoArrivingLaterIsUsedOnTheNextTick) {
    personAtPixel(*node, 1440.0, 7.0, t(1.0));
    node->tick(t(1.1));
    node->onCameraInfo(cameraInfo());
    personAtPixel(*node, 1440.0, 7.0, t(1.2));
    node->tick(t(1.3));
    EXPECT_NEAR(fake.velocities.back().forward, 0.35F, 1e-4);
    EXPECT_FALSE(node->searching());
}

TEST_F(NodeTest, UnusableCameraInfoIsNotUsed) {
    auto c = cameraInfo();
    c.k[0] = 0.0;
    node->onCameraInfo(c);
    personAtPixel(*node, 1440.0, 7.0, t(1.0));
    node->tick(t(1.1));
    // Not used: no target, so the scan runs (it used to hold still; that branch is gone).
    EXPECT_TRUE(node->searching());
    EXPECT_NEAR(fake.velocities.back().yaw, 25.0F, 1e-6);
}

TEST_F(NodeTest, BrokenTrackingIsIgnored) {
    node->onCameraInfo(cameraInfo());
    for (const double bad : {std::nan(""), std::numeric_limits<double>::infinity()}) {
        auto m = trackingMsg(6.0, 0.0);
        m.latitude_deg = bad;
        node->onTracking(m, t(1.0));
        m = trackingMsg(6.0, 0.0);
        m.altitude_m = bad;
        node->onTracking(m, t(1.0));
    }
    node->onGeolocation(droneMsg(), t(1.0));
    node->tick(t(1.1));
    EXPECT_TRUE(node->searching());  // never saw a valid target
}

TEST_F(NodeTest, BrokenTrackingDoesNotRefreshTheStoredTarget) {
    node->onCameraInfo(cameraInfo());
    personAtPixel(*node, 960.0, 6.0, t(1.0));
    auto m = trackingMsg(6.0, 0.0);
    m.longitude_deg = std::nan("");
    node->onTracking(m, t(1.7));  // ignored: neither position nor receipt time change
    node->onGeolocation(droneMsg(), t(1.7));
    node->tick(t(2.0));           // the last valid tracking is 1.0 s old
    EXPECT_TRUE(node->searching());
}

TEST_F(NodeTest, BrokenGeolocationIsIgnored) {
    node->onCameraInfo(cameraInfo());
    node->onTracking(trackingMsg(6.0, 0.0), t(1.0));
    auto m = droneMsg();
    m.latitude_deg = std::nan("");
    node->onGeolocation(m, t(1.0));
    m = droneMsg();
    m.longitude_deg = std::numeric_limits<double>::infinity();
    node->onGeolocation(m, t(1.0));
    m = droneMsg();
    m.altitude_m = std::nan("");
    node->onGeolocation(m, t(1.0));
    node->tick(t(1.1));
    EXPECT_TRUE(node->searching());  // never saw a valid drone position
}

TEST_F(NodeTest, BrokenGeolocationDoesNotRefreshTheStoredPosition) {
    node->onCameraInfo(cameraInfo());
    personAtPixel(*node, 960.0, 6.0, t(1.0));
    auto m = droneMsg();
    m.latitude_deg = std::nan("");
    node->onGeolocation(m, t(1.7));
    node->onTracking(trackingMsg(6.0, 0.0), t(1.7));
    node->tick(t(2.0));  // the last valid geolocation is 1.0 s old
    EXPECT_TRUE(node->searching());
}

TEST_F(NodeTest, LargeTrackingErrorIsIgnoredWhenGated) {
    FakeState f;
    auto n = make({{"max_target_error_m", 5.0}}, &f);
    n->onCameraInfo(cameraInfo());
    n->onGeolocation(droneMsg(), t(1.0));
    auto m = trackingMsg(6.0, 0.0);
    m.error_x_m = 10.0;
    n->onTracking(m, t(1.0));
    n->tick(t(1.1));
    EXPECT_TRUE(n->searching());  // 10 m error against a 5 m gate: not a sighting
    m.error_x_m = 3.0;
    m.error_y_m = 4.0;            // norm exactly 5 m: accepted
    n->onTracking(m, t(1.2));
    n->tick(t(1.3));
    EXPECT_FALSE(n->searching());
    m.error_z_m = 0.1;            // just above the gate: ignored, the earlier fix goes stale
    n->onTracking(m, t(2.0));
    n->onGeolocation(droneMsg(), t(2.0));
    n->tick(t(2.5));              // 1.3 s since the accepted message
    EXPECT_TRUE(n->searching());
}

TEST_F(NodeTest, ErrorGateOffByDefaultAcceptsAnyError) {
    node->onCameraInfo(cameraInfo());
    node->onGeolocation(droneMsg(), t(1.0));
    auto m = trackingMsg(6.0, 0.0);
    m.error_x_m = 1e6;
    m.error_y_m = 1e6;
    m.error_z_m = 1e6;
    node->onTracking(m, t(1.0));
    node->tick(t(1.1));
    EXPECT_FALSE(node->searching());
    // Negative or non-finite errors are still broken data.
    m.error_x_m = -1.0;
    node->onTracking(m, t(5.0));
    node->onGeolocation(droneMsg(), t(5.0));
    node->tick(t(5.1));
    EXPECT_TRUE(node->searching());
}

TEST_F(NodeTest, FailedVelocityCommandDoesNotCrash) {
    fake.velocity_result = {false, "link"};
    node->onCameraInfo(cameraInfo());
    personAtPixel(*node, 960.0, 6.0, t(1.0));
    EXPECT_NO_THROW(node->tick(t(1.1)));
}

TEST_F(NodeTest, ParametersCannotExceedHardCaps) {
    FakeState f;
    auto n = make({{"max_forward_m_s", 99.0}, {"max_strafe_m_s", 99.0},
                   {"max_yaw_rate_deg_s", 9999.0}},
                  &f);
    n->onCameraInfo(cameraInfo());
    personAtPixel(*n, 1920.0, 9.0, t(1.0));
    n->tick(t(1.1));
    ASSERT_EQ(f.velocities.size(), 1U);
    EXPECT_LE(f.velocities[0].forward, kForwardSpeedCeilingMps + 1e-6);
    EXPECT_LE(f.velocities[0].right, kStrafeSpeedCeilingMps + 1e-6);
    EXPECT_LE(f.velocities[0].yaw, kYawRateCeilingDegS + 1e-6);
}

// ---------- tracked position, drone position and heading ----------

TEST_F(NodeTest, FollowsFromTrackingGeolocationAndHeading) {
    node->onCameraInfo(cameraInfo());
    // Person 7 m east of a drone that faces east: dead ahead and 1 m too far.
    fake.heading_deg = 90.0;
    node->onTracking(trackingMsg(0.0, 7.0), t(1.0));
    node->onGeolocation(droneMsg(), t(1.0));
    node->tick(t(1.1));
    ASSERT_EQ(fake.velocities.size(), 1U);
    EXPECT_FALSE(node->searching());
    EXPECT_NEAR(fake.velocities[0].forward, 0.35F, 1e-4);
    EXPECT_NEAR(fake.velocities[0].right, 0.0F, 1e-4);
    EXPECT_NEAR(fake.velocities[0].yaw, 0.0F, 1e-3);
    EXPECT_FLOAT_EQ(fake.velocities[0].down, 0.0F);
}

TEST_F(NodeTest, HeadingTurnsTheTargetInTheImage) {
    node->onCameraInfo(cameraInfo());
    // Person 7 m due north. Facing north it is centred; facing 30 deg to the right of north it is
    // 30 deg to the left of the nose, so the drone must yaw and strafe left.
    node->onTracking(trackingMsg(7.0, 0.0), t(1.0));
    node->onGeolocation(droneMsg(), t(1.0));
    fake.heading_deg = 0.0;
    node->tick(t(1.1));
    fake.heading_deg = 30.0;
    node->onTracking(trackingMsg(7.0, 0.0), t(1.15));
    node->onGeolocation(droneMsg(), t(1.15));
    node->tick(t(1.2));
    fake.heading_deg = 330.0;  // same, the other way round
    node->onTracking(trackingMsg(7.0, 0.0), t(1.25));
    node->onGeolocation(droneMsg(), t(1.25));
    node->tick(t(1.3));
    ASSERT_EQ(fake.velocities.size(), 3U);
    EXPECT_NEAR(fake.velocities[0].yaw, 0.0F, 1e-3);
    EXPECT_LT(fake.velocities[1].yaw, -1.0F);
    EXPECT_LT(fake.velocities[1].right, 0.0F);
    EXPECT_GT(fake.velocities[2].yaw, 1.0F);
    EXPECT_GT(fake.velocities[2].right, 0.0F);
    // x = 960 + 1000 * tan(-30 deg) = 383 px: normalised -0.6 of the half width -> yaw -0.60 * 45.
    EXPECT_NEAR(fake.velocities[1].yaw, -std::abs(std::tan(30.0 / kDegPerRad)) * 1000.0 / 960.0 * 45.0,
                1e-2);
}

TEST_F(NodeTest, NoTrackingMeansTheScanNotAFollowCommand) {
    node->onCameraInfo(cameraInfo());
    node->onGeolocation(droneMsg(), t(1.0));  // the drone is known, the person never was
    node->tick(t(1.1));
    ASSERT_EQ(fake.velocities.size(), 1U);  // the scan does send commands
    EXPECT_TRUE(node->searching());
    EXPECT_NEAR(fake.velocities[0].forward, 0.28F, 1e-6);
    EXPECT_NEAR(fake.velocities[0].right, 0.0F, 1e-6);
    EXPECT_NEAR(fake.velocities[0].yaw, 25.0F, 1e-6);
}

TEST_F(NodeTest, TrackingWithoutGeolocationMeansSearch) {
    node->onCameraInfo(cameraInfo());
    node->onTracking(trackingMsg(7.0, 0.0), t(1.0));
    node->tick(t(1.1));
    ASSERT_EQ(fake.velocities.size(), 1U);
    EXPECT_TRUE(node->searching());
    EXPECT_NEAR(fake.velocities[0].forward, 0.28F, 1e-6);
    EXPECT_NEAR(fake.velocities[0].yaw, 25.0F, 1e-6);
}

TEST_F(NodeTest, MissingHeadingMeansSearch) {
    node->onCameraInfo(cameraInfo());
    fake.heading_deg = std::nullopt;
    personAtPixel(*node, 1440.0, 7.0, t(1.0));
    node->tick(t(1.1));
    ASSERT_EQ(fake.velocities.size(), 1U);
    EXPECT_TRUE(node->searching());
    EXPECT_NEAR(fake.velocities[0].forward, 0.28F, 1e-6);
    EXPECT_NEAR(fake.velocities[0].yaw, 25.0F, 1e-6);
    fake.heading_deg = std::nan("");  // a non-finite heading is as good as none
    personAtPixel(*node, 1440.0, 7.0, t(1.2));
    node->tick(t(1.3));
    EXPECT_TRUE(node->searching());
    fake.heading_deg = 0.0;  // heading back: the target is followed again
    personAtPixel(*node, 1440.0, 7.0, t(1.4));
    node->tick(t(1.5));
    EXPECT_FALSE(node->searching());
    EXPECT_NEAR(fake.velocities.back().forward, 0.35F, 1e-4);
}

TEST_F(NodeTest, StaleTrackingStartsTheSearch) {
    node->onCameraInfo(cameraInfo());
    node->onTracking(trackingMsg(6.0, 0.0), t(1.0));
    node->onGeolocation(droneMsg(), t(1.9));  // the drone position is fresh, the person is not
    node->tick(t(1.9));                        // 0.9 s old: past the 800 ms timeout
    EXPECT_TRUE(node->searching());
    EXPECT_NEAR(fake.velocities.back().yaw, 25.0F, 1e-6);
}

TEST_F(NodeTest, StaleGeolocationStartsTheSearch) {
    node->onCameraInfo(cameraInfo());
    node->onGeolocation(droneMsg(), t(1.0));
    node->onTracking(trackingMsg(6.0, 0.0), t(1.9));  // the person is fresh, the drone is not
    node->tick(t(1.9));
    EXPECT_TRUE(node->searching());
    EXPECT_NEAR(fake.velocities.back().yaw, 25.0F, 1e-6);
}

TEST_F(NodeTest, FreshnessUsesReceiptTimeNotTheHeaderStamp) {
    node->onCameraInfo(cameraInfo());
    // Header stamps from the far future are old news once the receipt time is old.
    auto tr = trackingMsg(6.0, 0.0);
    auto geo = droneMsg();
    tr.header.stamp = rclcpp::Time(1000, 0, RCL_ROS_TIME);
    geo.header.stamp = rclcpp::Time(1000, 0, RCL_ROS_TIME);
    node->onTracking(tr, t(1.0));
    node->onGeolocation(geo, t(1.0));
    node->tick(t(3.0));
    EXPECT_TRUE(node->searching());
    // Header stamps from 1970 are fine when the messages were just received.
    tr.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    geo.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    node->onTracking(tr, t(3.5));
    node->onGeolocation(geo, t(3.5));
    node->tick(t(3.6));
    EXPECT_FALSE(node->searching());
}

TEST_F(NodeTest, TargetBehindTheDroneOnlyTurns) {
    node->onCameraInfo(cameraInfo());
    for (const double rel : {180.0, 170.0, -170.0, 120.0}) {
        fake.velocities.clear();
        personAt(*node, rel, 12.0, t(1.0));  // far: a followed target would drive forward
        node->tick(t(1.1));
        ASSERT_EQ(fake.velocities.size(), 1U) << "bearing " << rel;
        EXPECT_FALSE(node->searching()) << "bearing " << rel;
        EXPECT_FLOAT_EQ(fake.velocities[0].forward, 0.0F) << "bearing " << rel;
        if (rel > 0.0) {
            EXPECT_GT(fake.velocities[0].yaw, 0.0F) << "bearing " << rel;
        } else {
            EXPECT_LT(fake.velocities[0].yaw, 0.0F) << "bearing " << rel;
        }
    }
}

TEST_F(NodeTest, TargetOutsideTheCameraViewGivesNoForwardMotion) {
    // Replaces the clipped-box tests: a target that is not in the image has no usable range.
    node->onCameraInfo(cameraInfo());
    personAt(*node, 60.0, 12.0, t(1.0));  // x = 960 + 1000 * tan(60 deg) = 2692 px, off the frame
    node->tick(t(1.1));
    ASSERT_EQ(fake.velocities.size(), 1U);
    EXPECT_FLOAT_EQ(fake.velocities[0].forward, 0.0F);
    EXPECT_GT(fake.velocities[0].yaw, 0.0F);  // still turns toward it
    personAt(*node, 40.0, 12.0, t(1.2));  // 1799 px: in view, so the range counts
    node->tick(t(1.3));
    EXPECT_GT(fake.velocities.back().forward, 0.0F);
}

TEST_F(NodeTest, FollowModeNeverStrafesWhenTheTargetHasNoRange) {
    // Beside or behind a close drone: outside the image, so no range and min_distance_m cannot be
    // enforced. The drone only yaws toward the person.
    node->onCameraInfo(cameraInfo());
    for (const double rel : {90.0, -90.0, 75.0, 135.0, 180.0, -150.0}) {
        for (const double range : {0.6, 1.0, 3.0}) {
            fake.velocities.clear();
            personAt(*node, rel, range, t(1.0));
            node->tick(t(1.1));
            ASSERT_EQ(fake.velocities.size(), 1U) << rel << " deg " << range << " m";
            EXPECT_FLOAT_EQ(fake.velocities[0].right, 0.0F) << rel << " deg " << range << " m";
            EXPECT_FLOAT_EQ(fake.velocities[0].forward, 0.0F) << rel << " deg " << range << " m";
            if (rel > 0.0 && rel < 180.0) {
                EXPECT_GT(fake.velocities[0].yaw, 0.0F) << rel << " deg " << range << " m";
            } else if (rel < 0.0) {
                EXPECT_LT(fake.velocities[0].yaw, 0.0F) << rel << " deg " << range << " m";
            }
            EXPECT_NE(fake.velocities[0].yaw, 0.0F) << rel << " deg " << range << " m";
        }
    }
}

TEST_F(NodeTest, InViewTargetStillStrafesInFollowMode) {
    // The out-of-view rule must not touch targets that have a range.
    node->onCameraInfo(cameraInfo());
    personAtPixel(*node, 1440.0, 7.0, t(1.0));
    node->tick(t(1.1));
    ASSERT_EQ(fake.velocities.size(), 1U);
    EXPECT_NEAR(fake.velocities[0].right, 0.1575F, 1e-5);
}

TEST_F(NodeTest, OrbitModeIsUnchangedForTargetsWithoutRange) {
    // Same geometry as above, orbit on: the out-of-view rule applies to follow mode only. The
    // reference value is what the unchanged orbit controller gives for the same target.
    FakeState f;
    auto n = make({{"orbit_enabled", true}}, &f);
    n->onCameraInfo(cameraInfo());
    personAt(*n, 90.0, 1.0, t(1.0));
    n->tick(t(1.1));
    ASSERT_EQ(f.velocities.size(), 1U);

    // The orbit controller itself gives no strafe without a range; the node must not add any.
    EXPECT_FLOAT_EQ(f.velocities[0].forward, 0.0F);
    EXPECT_FLOAT_EQ(f.velocities[0].right, 0.0F);
    EXPECT_GT(f.velocities[0].yaw, 0.0F);
    // And the same orbit node still circles a person that is in view (unchanged).
    f.velocities.clear();
    personAtPixel(*n, 960.0, 6.0, t(1.2));
    n->tick(t(1.3));
    ASSERT_EQ(f.velocities.size(), 1U);
    EXPECT_GT(f.velocities[0].right, 0.0F);
}

TEST_F(NodeTest, HeadingTimeoutIsReadByMainNotByTheNode) {
    // main.cpp reads heading_timeout_s on its own params node (the sanitize helper is tested in
    // test_heading_freshness); the follow node must not declare it.
    EXPECT_FALSE(node->has_parameter("heading_timeout_s"));
}

TEST_F(NodeTest, PersonDirectlyBelowHoldsStill) {
    node->onCameraInfo(cameraInfo());
    node->onTracking(trackingMsg(0.0, 0.0, 0.0), t(1.0));  // same lat/lon, 10 m lower
    node->onGeolocation(droneMsg(10.0), t(1.0));
    node->tick(t(1.1));
    ASSERT_EQ(fake.velocities.size(), 1U);
    EXPECT_FALSE(node->searching());
    EXPECT_FLOAT_EQ(fake.velocities[0].forward, 0.0F);
    EXPECT_FLOAT_EQ(fake.velocities[0].right, 0.0F);
    EXPECT_FLOAT_EQ(fake.velocities[0].yaw, 0.0F);
}

TEST_F(NodeTest, LostLinkWhileFollowingSendsNothingAndRestartsOffboard) {
    node->onCameraInfo(cameraInfo());
    personAtPixel(*node, 1440.0, 7.0, t(1.0));
    node->tick(t(1.1));
    ASSERT_EQ(fake.velocities.size(), 1U);
    ASSERT_TRUE(node->offboardActive());
    fake.state.reset();  // the link drops while the target is still fresh
    personAtPixel(*node, 1440.0, 7.0, t(1.2));
    node->tick(t(1.3));
    node->tick(t(1.4));
    EXPECT_EQ(fake.velocities.size(), 1U);
    EXPECT_FALSE(node->offboardActive());
    fake.state = flying();
    personAtPixel(*node, 1440.0, 7.0, t(2.0));
    node->tick(t(2.1));
    EXPECT_EQ(fake.offboard_starts, 2);
    ASSERT_EQ(fake.velocities.size(), 2U);
    EXPECT_NEAR(fake.velocities[1].forward, 0.35F, 1e-4);
}

// One limit per output, for targets all around the drone at every range, follow and orbit, default
// and hostile parameters.
TEST_F(NodeTest, LimitsHoldForTrackedTargetsAllAroundTheDrone) {
    struct Config {
        std::vector<rclcpp::Parameter> params;
        double forward, strafe, yaw;
    };
    const std::vector<Config> configs = {
        {{}, 0.7, 0.45, 45.0},
        {{{"orbit_enabled", true}}, 0.7, kOrbitSpeedCeilingMps, 45.0},
        {{{"max_forward_m_s", 99.0}, {"max_strafe_m_s", 99.0}, {"max_yaw_rate_deg_s", 9999.0}},
         kForwardSpeedCeilingMps, kStrafeSpeedCeilingMps, kYawRateCeilingDegS},
        {{{"orbit_enabled", true}, {"orbit_period_s", 5.0}, {"max_orbit_speed_m_s", 99.0},
          {"max_forward_m_s", 99.0}, {"max_yaw_rate_deg_s", 9999.0}},
         kForwardSpeedCeilingMps, kOrbitSpeedCeilingMps, kYawRateCeilingDegS},
    };
    for (const auto& cfg : configs) {
        FakeState f;
        auto n = make(cfg.params, &f);
        n->onCameraInfo(cameraInfo());
        double time = 1.0;
        size_t expected = 0;
        for (const double heading : {0.0, 137.0}) {
            f.heading_deg = heading;
            for (double rel = -180.0; rel < 180.0; rel += 7.5) {
                for (const double range : {0.0, 0.3, 0.5, 1.0, 4.0, 6.0, 9.0, 20.0, 500.0}) {
                    personAt(*n, rel, range, t(time), heading);
                    n->tick(t(time + 0.05));
                    time += 0.1;
                    ++expected;
                    ASSERT_EQ(f.velocities.size(), expected)
                        << "bearing " << rel << " range " << range;
                    const Velocity& v = f.velocities.back();
                    EXPECT_FALSE(n->searching());
                    EXPECT_LE(std::abs(v.forward), cfg.forward + 1e-6)
                        << "bearing " << rel << " range " << range;
                    EXPECT_LE(std::abs(v.right), cfg.strafe + 1e-6)
                        << "bearing " << rel << " range " << range;
                    EXPECT_LE(std::abs(v.yaw), cfg.yaw + 1e-6)
                        << "bearing " << rel << " range " << range;
                    EXPECT_FLOAT_EQ(v.down, 0.0F);
                }
            }
        }
    }
}

// ---------- smoothing ----------

TEST_F(NodeTest, AccelerationLimitsSmoothTheCommandsAndRestartFromRest) {
    FakeState f;
    auto n = make({{"max_forward_accel_m_s2", 1.0}, {"max_yaw_accel_deg_s2", 100.0}}, &f);
    n->onCameraInfo(cameraInfo());
    for (int i = 0; i < 3; ++i) {
        personAtPixel(*n, 1920.0, 9.0, t(1.0 + i * 0.1));  // far, at the right edge
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
    personAtPixel(*n, 1440.0, 7.0, t(1.0));  // half right, 1 m too far
    n->tick(t(1.1));
    ASSERT_EQ(f.velocities.size(), 1U);
    EXPECT_NEAR(f.velocities[0].forward, 0.7F, 1e-4);
    EXPECT_NEAR(f.velocities[0].yaw, 11.25F, 1e-4);
}

// ---------- orbit ----------

TEST_F(NodeTest, FollowsRatherThanOrbitsByDefault) {
    node->onCameraInfo(cameraInfo());
    personAtPixel(*node, 960.0, 6.0, t(1.0));
    node->tick(t(1.1));
    ASSERT_EQ(fake.velocities.size(), 1U);
    EXPECT_NEAR(fake.velocities[0].right, 0.0F, 1e-6);  // no sideways motion when centred
}

TEST_F(NodeTest, OrbitModeCirclesTheCentredPerson) {
    FakeState f;
    auto n = make({{"orbit_enabled", true}}, &f);
    n->onCameraInfo(cameraInfo());
    personAtPixel(*n, 960.0, 6.0, t(1.0));
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
    personAtPixel(*n, 960.0, 6.0, t(1.0));
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
    personAtPixel(*n, 960.0, 9.0, t(1.0));
    n->tick(t(1.1));
    ASSERT_EQ(f.velocities.size(), 1U);
    EXPECT_LE(std::abs(f.velocities[0].right), kOrbitSpeedCeilingMps + 1e-6);
}

TEST_F(NodeTest, LosingTheTargetWhileOrbitingFallsBackToTheScan) {
    FakeState f;
    auto n = make({{"orbit_enabled", true}}, &f);
    n->onCameraInfo(cameraInfo());
    personAtPixel(*n, 960.0, 6.0, t(1.0));
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
        personAtPixel(*n, 1440.0, 6.0, t(1.0 + i * 0.1));
        n->tick(t(1.05 + i * 0.1));
    }
    n->tick(t(5.0));  // target lost, scan runs
    personAtPixel(*n, 1440.0, 6.0, t(6.0));
    n->tick(t(6.05));
    const float first_after = f.velocities.back().yaw;
    f.velocities.clear();
    FakeState g;
    auto fresh = make({{"orbit_enabled", true}}, &g);
    fresh->onCameraInfo(cameraInfo());
    personAtPixel(*fresh, 1440.0, 6.0, t(1.0));
    fresh->tick(t(1.05));
    EXPECT_NEAR(first_after, g.velocities.back().yaw, 1e-4);  // same as a brand-new controller
}

// ---------- target lost: search around the last known direction ----------

TEST_F(NodeTest, LostTargetSweepsInPlaceTowardTheSideItWasLastSeen) {
    FakeState f;
    auto n = make({{"last_known_search_s", 8.0}}, &f);
    n->onCameraInfo(cameraInfo());
    personAtPixel(*n, 1500.0, 6.0, t(1.0));  // right of centre
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
    personAtPixel(*n, 400.0, 6.0, t(1.0));
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
    personAtPixel(*n, 960.0, 6.0, t(1.0));
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
    personAtPixel(*node, 1500.0, 6.0, t(1.0));
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
    personAtPixel(*n, 960.0, 6.0, t(1.0));
    n->tick(t(1.05));
    n->tick(t(2.5));
    EXPECT_TRUE(n->searching());
    personAtPixel(*n, 960.0, 7.0, t(3.0));
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
    personAtPixel(*node, 960.0, 6.0, t(1.0));
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
    personAtPixel(*node, 960.0, 6.0, t(3.0));
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
    personAtPixel(*n, 960.0, 6.0, t(10.0));
    n->tick(t(10.1));
    EXPECT_EQ(f.offboard_starts, 2);           // target seen: follow again
    EXPECT_GT(f.velocities.size(), sent);
}
