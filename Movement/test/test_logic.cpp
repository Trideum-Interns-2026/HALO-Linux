#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "movement/follow_logic.hpp"

using namespace movement;

namespace {

CameraGeometry camera() {
    CameraGeometry c;
    c.center_x_px = 960.0;
    c.width_px = 1920.0;
    c.focal_x_px = 1000.0;
    return c;
}

const FollowParams kP = sanitize(FollowParams{});

}  // namespace

// ---------- defaults are the original controller's numbers ----------

TEST(Defaults, MatchTheOriginalController) {
    const FollowParams p;
    EXPECT_DOUBLE_EQ(p.safe_distance_m, 6.0);
    EXPECT_DOUBLE_EQ(p.min_distance_m, 4.0);
    EXPECT_DOUBLE_EQ(p.max_distance_m, 9.0);
    EXPECT_DOUBLE_EQ(p.max_forward_m_s, 0.7);
    EXPECT_DOUBLE_EQ(p.max_strafe_m_s, 0.45);
    EXPECT_DOUBLE_EQ(p.max_yaw_rate_deg_s, 45.0);
    EXPECT_DOUBLE_EQ(p.search_yaw_rate_deg_s, 25.0);
    EXPECT_DOUBLE_EQ(p.search_forward_m_s, 0.35);
    EXPECT_DOUBLE_EQ(p.search_strafe_m_s, 0.45);
    EXPECT_DOUBLE_EQ(p.search_orbit_hz, 0.25);
}

TEST(Defaults, SanitizeLeavesTheDefaultsUntouched) {
    const FollowParams s = sanitize(FollowParams{});
    const FollowParams p;
    EXPECT_DOUBLE_EQ(s.safe_distance_m, p.safe_distance_m);
    EXPECT_DOUBLE_EQ(s.max_forward_m_s, p.max_forward_m_s);
    EXPECT_DOUBLE_EQ(s.max_strafe_m_s, p.max_strafe_m_s);
    EXPECT_DOUBLE_EQ(s.max_yaw_rate_deg_s, p.max_yaw_rate_deg_s);
    EXPECT_DOUBLE_EQ(s.search_forward_m_s, p.search_forward_m_s);
    EXPECT_DOUBLE_EQ(s.search_strafe_m_s, p.search_strafe_m_s);
    EXPECT_DOUBLE_EQ(s.search_yaw_rate_deg_s, p.search_yaw_rate_deg_s);
    EXPECT_DOUBLE_EQ(s.search_orbit_hz, p.search_orbit_hz);
}

// ---------- follow: centring (yaw and strafe) ----------

struct OffsetCase {
    double x_px;
    double yaw_deg_s;
    double right_m_s;
};

class Offset : public ::testing::TestWithParam<OffsetCase> {};

TEST_P(Offset, YawAndStrafeFollowTheOriginalFormula) {
    const OffsetCase c = GetParam();
    const VelocityCommand v = computeFollowCommand(c.x_px, 6.0, camera(), kP);
    EXPECT_NEAR(v.yawspeed_deg_s, c.yaw_deg_s, 1e-9);
    EXPECT_NEAR(v.right_m_s, c.right_m_s, 1e-9);
}

INSTANTIATE_TEST_SUITE_P(
    Frame, Offset,
    ::testing::Values(
        OffsetCase{960.0, 0.0, 0.0},                 // centred
        OffsetCase{1440.0, 22.5, 0.7 * 0.5 * 0.45},  // half way to the right edge
        OffsetCase{480.0, -22.5, -0.7 * 0.5 * 0.45},
        OffsetCase{1920.0, 45.0, 0.7 * 0.45},        // right edge: full yaw, 0.7 of max strafe
        OffsetCase{0.0, -45.0, -0.7 * 0.45},
        OffsetCase{5000.0, 45.0, 0.7 * 0.45},        // beyond the frame clamps
        OffsetCase{-5000.0, -45.0, -0.7 * 0.45}));

TEST(Follow, TargetRightOfCentreYawsClockwiseAndStrafesRight) {
    const VelocityCommand v = computeFollowCommand(1200.0, 6.0, camera(), kP);
    EXPECT_GT(v.yawspeed_deg_s, 0.0);
    EXPECT_GT(v.right_m_s, 0.0);
}

// ---------- follow: distance ----------

struct DistanceCase {
    double distance_m;
    double forward_m_s;
};

class Distance : public ::testing::TestWithParam<DistanceCase> {};

TEST_P(Distance, ForwardSpeedFollowsTheOriginalFormula) {
    const DistanceCase c = GetParam();
    const VelocityCommand v = computeFollowCommand(960.0, c.distance_m, camera(), kP);
    EXPECT_NEAR(v.forward_m_s, c.forward_m_s, 1e-9);
}

INSTANTIATE_TEST_SUITE_P(
    Range, Distance,
    ::testing::Values(
        DistanceCase{6.0, 0.0},            // at the safe distance
        DistanceCase{7.0, 0.35},           // 1 m too far
        DistanceCase{5.0, -0.35},          // 1 m too close
        DistanceCase{9.0, 0.7},            // 3 m * 0.35 = 1.05, capped at max forward
        DistanceCase{4.0, -0.7},
        DistanceCase{100.0, 0.7},          // distance is clamped to [min, max] first
        DistanceCase{0.5, -0.7}));

TEST(Follow, NoRangeMeansNoForwardMotion) {
    const VelocityCommand v = computeFollowCommand(1440.0, std::nullopt, camera(), kP);
    EXPECT_DOUBLE_EQ(v.forward_m_s, 0.0);
    EXPECT_GT(v.yawspeed_deg_s, 0.0);  // still centres
}

TEST(Follow, EveryOutputStaysWithinTheConfiguredLimits) {
    for (double x = -500.0; x <= 2500.0; x += 37.0) {
        for (double d = 0.1; d <= 60.0; d += 1.3) {
            const VelocityCommand v = computeFollowCommand(x, d, camera(), kP);
            EXPECT_LE(std::abs(v.forward_m_s), kP.max_forward_m_s + 1e-12);
            EXPECT_LE(std::abs(v.right_m_s), kP.max_strafe_m_s + 1e-12);
            EXPECT_LE(std::abs(v.yawspeed_deg_s), kP.max_yaw_rate_deg_s + 1e-12);
        }
    }
}

TEST(Follow, TighterLimitsAreHonoured) {
    FollowParams p;
    p.max_forward_m_s = 0.2;
    p.max_strafe_m_s = 0.1;
    p.max_yaw_rate_deg_s = 10.0;
    p = sanitize(p);
    const VelocityCommand v = computeFollowCommand(1920.0, 9.0, camera(), p);
    EXPECT_DOUBLE_EQ(v.forward_m_s, 0.2);
    EXPECT_NEAR(v.right_m_s, 0.07, 1e-12);
    EXPECT_DOUBLE_EQ(v.yawspeed_deg_s, 10.0);
}

// ---------- search spiral ----------

TEST(Search, StartsCreepingForwardWithTheOriginalNumbers) {
    const VelocityCommand v = computeSearchCommand(0.0, kP);
    EXPECT_NEAR(v.forward_m_s, 0.35 * 0.8, 1e-12);
    EXPECT_NEAR(v.right_m_s, 0.0, 1e-12);
    EXPECT_DOUBLE_EQ(v.yawspeed_deg_s, 25.0);
}

TEST(Search, StrafeIsASineAtTheOrbitFrequency) {
    // 0.25 Hz: period 4 s.
    EXPECT_NEAR(computeSearchCommand(1.0, kP).right_m_s, 0.45, 1e-9);    // peak
    EXPECT_NEAR(computeSearchCommand(2.0, kP).right_m_s, 0.0, 1e-9);
    EXPECT_NEAR(computeSearchCommand(3.0, kP).right_m_s, -0.45, 1e-9);   // trough
    EXPECT_NEAR(computeSearchCommand(4.0, kP).right_m_s, 0.0, 1e-9);
}

TEST(Search, YawRateAndForwardSpeedStayConstant) {
    for (double t : {0.0, 0.7, 5.0, 123.4}) {
        const VelocityCommand v = computeSearchCommand(t, kP);
        EXPECT_DOUBLE_EQ(v.forward_m_s, 0.35 * 0.8);
        EXPECT_DOUBLE_EQ(v.yawspeed_deg_s, 25.0);
    }
}

TEST(Search, ZeroOrbitFrequencyMeansNoSidewaysMotion) {
    FollowParams p;
    p.search_orbit_hz = 0.0;
    p = sanitize(p);
    for (double t : {0.0, 1.0, 9.0}) {
        EXPECT_NEAR(computeSearchCommand(t, p).right_m_s, 0.0, 1e-12);
    }
}

TEST(Search, ParametersCannotMakeTheScanFasterThanTheFollowLimits) {
    FollowParams p;
    p.search_forward_m_s = 50.0;
    p.search_strafe_m_s = 50.0;
    p.search_yaw_rate_deg_s = 500.0;
    p = sanitize(p);
    for (double t = 0.0; t < 8.0; t += 0.25) {
        const VelocityCommand v = computeSearchCommand(t, p);
        EXPECT_LE(std::abs(v.forward_m_s), p.max_forward_m_s + 1e-12);
        EXPECT_LE(std::abs(v.right_m_s), p.max_strafe_m_s + 1e-12);
        EXPECT_LE(std::abs(v.yawspeed_deg_s), p.max_yaw_rate_deg_s + 1e-12);
    }
}

// ---------- hard caps ----------

TEST(Sanitize, EveryLimitIsEnforced) {
    FollowParams wild;
    wild.min_distance_m = 0.1;
    wild.max_distance_m = 500.0;
    wild.safe_distance_m = 1000.0;
    wild.max_forward_m_s = 50.0;
    wild.max_strafe_m_s = 50.0;
    wild.max_yaw_rate_deg_s = 5000.0;
    wild.search_orbit_hz = 40.0;
    const FollowParams s = sanitize(wild);
    EXPECT_DOUBLE_EQ(s.min_distance_m, kMinDistanceFloorM);
    EXPECT_DOUBLE_EQ(s.max_distance_m, kMaxDistanceCeilingM);
    EXPECT_DOUBLE_EQ(s.safe_distance_m, kMaxDistanceCeilingM);
    EXPECT_DOUBLE_EQ(s.max_forward_m_s, kForwardSpeedCeilingMps);
    EXPECT_DOUBLE_EQ(s.max_strafe_m_s, kStrafeSpeedCeilingMps);
    EXPECT_DOUBLE_EQ(s.max_yaw_rate_deg_s, kYawRateCeilingDegS);
    EXPECT_DOUBLE_EQ(s.search_orbit_hz, kSearchOrbitCeilingHz);
}

TEST(Sanitize, NegativeLimitsBecomeZeroNotReversed) {
    FollowParams p;
    p.max_forward_m_s = -1.0;
    p.max_strafe_m_s = -1.0;
    p.max_yaw_rate_deg_s = -1.0;
    const FollowParams s = sanitize(p);
    EXPECT_DOUBLE_EQ(s.max_forward_m_s, 0.0);
    EXPECT_DOUBLE_EQ(s.max_strafe_m_s, 0.0);
    EXPECT_DOUBLE_EQ(s.max_yaw_rate_deg_s, 0.0);
}

TEST(Sanitize, NonFiniteValuesFallBackToTheDefaults) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    FollowParams p;
    p.max_forward_m_s = nan;
    p.safe_distance_m = nan;
    p.search_yaw_rate_deg_s = std::numeric_limits<double>::infinity();
    const FollowParams s = sanitize(p);
    EXPECT_DOUBLE_EQ(s.max_forward_m_s, kMaxForwardSpeedMps);
    EXPECT_DOUBLE_EQ(s.safe_distance_m, kDefaultDistanceM);
    EXPECT_DOUBLE_EQ(s.search_yaw_rate_deg_s, kDefaultSearchYawRateDegS);
}

TEST(Sanitize, DistancesStayOrdered) {
    FollowParams p;
    p.min_distance_m = 12.0;
    p.max_distance_m = 5.0;
    p.safe_distance_m = 1.0;
    const FollowParams s = sanitize(p);
    EXPECT_LE(s.min_distance_m, s.safe_distance_m);
    EXPECT_LE(s.safe_distance_m, s.max_distance_m);
}

// ---------- camera geometry ----------

TEST(Camera, UsableOnlyWithFiniteCentreAndPositiveFocalLengthAndWidth) {
    EXPECT_TRUE(isUsable(camera()));
    CameraGeometry c = camera();
    c.focal_x_px = 0.0;
    EXPECT_FALSE(isUsable(c));
    c = camera();
    c.width_px = 0.0;
    EXPECT_FALSE(isUsable(c));
    c = camera();
    c.center_x_px = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(isUsable(c));
}

// ---------- orbit mode ----------

namespace {
const OrbitParams kO = sanitize(OrbitParams{true, false, 30.0, 1.5});
constexpr double kLapSpeed = 2.0 * kPi * 6.0 / 30.0;  // one lap in 30 s at 6 m
constexpr double kDt = 0.1;                            // the node's 100 ms loop

VelocityCommand orbitOnce(double x_px, std::optional<double> d, const OrbitParams& o = kO,
                          OrbitState* state = nullptr) {
    OrbitState fresh;
    return computeOrbitCommand(x_px, d, camera(), kP, o, state != nullptr ? *state : fresh, kDt);
}

// Kinematic model of the drone circling a person at the origin with a fixed forward camera.
// x east, y north, heading clockwise from north (so a positive yaw rate turns right).
struct Sim {
    double x{0.0}, y{-6.0}, heading_deg{0.0};
    double distance() const { return std::hypot(x, y); }
    double bearing_deg() const {  // + = person to the right of the camera axis
        double b = std::atan2(-x, -y) * 180.0 / kPi - heading_deg;
        while (b > 180.0) b -= 360.0;
        while (b < -180.0) b += 360.0;
        return b;
    }
    double pixel_x() const {
        return camera().center_x_px + camera().focal_x_px * std::tan(bearing_deg() * kPi / 180.0);
    }
    void step(const VelocityCommand& c, double dt) {
        const double h = heading_deg * kPi / 180.0;
        x += (c.forward_m_s * std::sin(h) + c.right_m_s * std::cos(h)) * dt;
        y += (c.forward_m_s * std::cos(h) - c.right_m_s * std::sin(h)) * dt;
        heading_deg += c.yawspeed_deg_s * dt;
    }
};
}  // namespace

TEST(Orbit, DefaultsAreOffThirtySecondsCounterClockwise) {
    const OrbitParams o;
    EXPECT_FALSE(o.enabled);
    EXPECT_FALSE(o.clockwise);
    EXPECT_DOUBLE_EQ(o.period_s, 30.0);
    EXPECT_DOUBLE_EQ(o.max_speed_m_s, 1.5);
}

TEST(Orbit, SpeedIsCircumferenceOverPeriod) {
    EXPECT_NEAR(orbitSpeed(6.0, kO), kLapSpeed, 1e-12);
    EXPECT_NEAR(orbitSpeed(4.0, kO), 2.0 * kPi * 4.0 / 30.0, 1e-12);
}

TEST(Orbit, SpeedIsCappedAndTheLapSlowsDown) {
    const OrbitParams capped = sanitize(OrbitParams{true, false, 5.0, 1.0});
    EXPECT_DOUBLE_EQ(orbitSpeed(9.0, capped), 1.0);  // would be 11.3 m/s uncapped
}

TEST(Orbit, CounterClockwiseStrafesRightAndClockwiseLeft) {
    EXPECT_NEAR(orbitOnce(960.0, 6.0).right_m_s, kLapSpeed, 1e-9);
    OrbitParams cw = kO;
    cw.clockwise = true;
    EXPECT_NEAR(orbitOnce(960.0, 6.0, cw).right_m_s, -kLapSpeed, 1e-9);
}

TEST(Orbit, YawComesOnlyFromThePixelOffsetNotFromAGuessedTurnRate) {
    const VelocityCommand centred = orbitOnce(960.0, 6.0);
    EXPECT_DOUBLE_EQ(centred.yawspeed_deg_s, 0.0);  // person centred: no turning command at all
    const VelocityCommand right = orbitOnce(1440.0, 6.0);  // half way to the right edge
    EXPECT_NEAR(right.yawspeed_deg_s, 22.5 + kOrbitYawIntegralGain * 0.5 * kDt, 1e-9);
    EXPECT_GT(orbitOnce(1440.0, 6.0).yawspeed_deg_s, 0.0);   // turn toward the person
    EXPECT_LT(orbitOnce(480.0, 6.0).yawspeed_deg_s, 0.0);
}

TEST(Orbit, IntegralTermAccumulatesAndIsBounded) {
    OrbitState state;
    for (int i = 0; i < 1000; ++i) {
        (void)orbitOnce(1920.0, 6.0, kO, &state);  // person stuck at the right edge
    }
    EXPECT_LE(std::abs(state.yaw_integral), kP.max_yaw_rate_deg_s / kOrbitYawIntegralGain + 1e-12);
    EXPECT_NEAR(orbitOnce(1920.0, 6.0, kO, &state).yawspeed_deg_s, kP.max_yaw_rate_deg_s, 1e-9);
}

TEST(Orbit, BadTimeStepLeavesTheIntegralAlone) {
    OrbitState state;
    for (double dt : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN()}) {
        (void)computeOrbitCommand(1440.0, 6.0, camera(), kP, kO, state, dt);
        EXPECT_DOUBLE_EQ(state.yaw_integral, 0.0);
    }
}

TEST(Orbit, HoldsTheRadiusLikeFollowDoes) {
    EXPECT_NEAR(orbitOnce(960.0, 7.0).forward_m_s, 0.35, 1e-9);
    EXPECT_NEAR(orbitOnce(960.0, 5.0).forward_m_s, -0.35, 1e-9);
    EXPECT_NEAR(orbitOnce(960.0, 100.0).forward_m_s, 0.7, 1e-9);  // follow's forward limit applies
}

TEST(Orbit, WithoutARangeItOnlyKeepsTheTargetCentred) {
    const VelocityCommand c = orbitOnce(1440.0, std::nullopt);
    EXPECT_DOUBLE_EQ(c.forward_m_s, 0.0);
    EXPECT_DOUBLE_EQ(c.right_m_s, 0.0);
    EXPECT_GT(c.yawspeed_deg_s, 0.0);
}

TEST(Orbit, EveryOutputStaysWithinItsLimits) {
    for (double x : {-5000.0, 0.0, 960.0, 1920.0, 9000.0}) {
        for (double d : {0.1, 4.0, 6.0, 9.0, 500.0}) {
            for (bool cw : {false, true}) {
                const OrbitParams o = sanitize(OrbitParams{true, cw, 0.0, 1e9});  // hostile
                OrbitState state;
                for (int i = 0; i < 200; ++i) {
                    const VelocityCommand c =
                        computeOrbitCommand(x, d, camera(), kP, o, state, kDt);
                    ASSERT_LE(std::abs(c.forward_m_s), kP.max_forward_m_s + 1e-9);
                    ASSERT_LE(std::abs(c.right_m_s), kOrbitSpeedCeilingMps + 1e-9);
                    ASSERT_LE(std::abs(c.yawspeed_deg_s), kP.max_yaw_rate_deg_s + 1e-9);
                }
            }
        }
    }
}

TEST(Orbit, SanitizeEnforcesTheHardBounds) {
    const OrbitParams low = sanitize(OrbitParams{true, false, 0.0, -3.0});
    EXPECT_DOUBLE_EQ(low.period_s, kOrbitPeriodFloorS);
    EXPECT_DOUBLE_EQ(low.max_speed_m_s, 0.0);
    const OrbitParams high = sanitize(OrbitParams{true, false, 1e9, 1e9});
    EXPECT_DOUBLE_EQ(high.period_s, kOrbitPeriodCeilingS);
    EXPECT_DOUBLE_EQ(high.max_speed_m_s, kOrbitSpeedCeilingMps);
}

TEST(Orbit, SanitizeReplacesNonFiniteValuesWithTheDefaults) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const OrbitParams o = sanitize(OrbitParams{true, false, nan, nan});
    EXPECT_DOUBLE_EQ(o.period_s, kDefaultOrbitPeriodS);
    EXPECT_DOUBLE_EQ(o.max_speed_m_s, kDefaultOrbitSpeedMps);
}

// The closed-loop checks: drive the controller against a model of the drone and camera.

class OrbitLoop : public ::testing::TestWithParam<bool> {};

TEST_P(OrbitLoop, CameraKeepsThePersonCentredAtTheRadiusAroundTheWholeLap) {
    OrbitParams o = kO;
    o.clockwise = GetParam();
    Sim sim;
    OrbitState state;
    double worst_bearing = 0.0, worst_radius_error = 0.0, travelled_deg = 0.0;
    const double start_angle = std::atan2(-sim.x, -sim.y);
    double last_angle = start_angle;
    const int steps = static_cast<int>(60.0 / kDt);  // two laps
    for (int i = 0; i < steps; ++i) {
        const VelocityCommand c = computeOrbitCommand(sim.pixel_x(), sim.distance(), camera(), kP,
                                                      o, state, kDt);
        sim.step(c, kDt);
        if (i * kDt > 10.0) {  // after the transient
            worst_bearing = std::max(worst_bearing, std::abs(sim.bearing_deg()));
            worst_radius_error = std::max(worst_radius_error, std::abs(sim.distance() - 6.0));
        }
        double a = std::atan2(-sim.x, -sim.y);
        double delta = a - last_angle;
        while (delta > kPi) delta -= 2.0 * kPi;
        while (delta < -kPi) delta += 2.0 * kPi;
        travelled_deg += delta * 180.0 / kPi;
        last_angle = a;
    }
    EXPECT_LT(worst_bearing, 3.0) << "person drifted away from the image centre";
    EXPECT_LT(worst_radius_error, 0.6) << "radius not held";
    // Two laps in 60 s, in the requested direction (the angle seen from the person grows
    // clockwise-from-north for a clockwise orbit).
    EXPECT_NEAR(std::abs(travelled_deg), 720.0, 60.0);
    EXPECT_EQ(travelled_deg > 0.0, GetParam() == true);
}

INSTANTIATE_TEST_SUITE_P(BothDirections, OrbitLoop, ::testing::Bool());

TEST(OrbitLoop, PersonThatMovesStaysCentred) {
    Sim sim;
    OrbitState state;
    double worst_bearing = 0.0;
    double px = 0.0, py = 0.0;  // the person walks east at 0.5 m/s
    for (int i = 0; i < static_cast<int>(40.0 / kDt); ++i) {
        const double dx = px - sim.x, dy = py - sim.y;
        // Camera view of a person at (px, py): rebuild the bearing from the offset.
        double b = std::atan2(dx, dy) * 180.0 / kPi - sim.heading_deg;
        while (b > 180.0) b -= 360.0;
        while (b < -180.0) b += 360.0;
        const double x_px = camera().center_x_px +
                            camera().focal_x_px * std::tan(b * kPi / 180.0);
        const VelocityCommand c = computeOrbitCommand(x_px, std::hypot(dx, dy), camera(), kP, kO,
                                                      state, kDt);
        sim.step(c, kDt);
        px += 0.5 * kDt;
        if (i * kDt > 15.0) worst_bearing = std::max(worst_bearing, std::abs(b));
    }
    EXPECT_LT(worst_bearing, 8.0);
}

// ---------- tunable gains and command smoothing ----------

TEST(Gains, DefaultsAreTheOriginalControllersNumbers) {
    const FollowParams p;
    EXPECT_DOUBLE_EQ(p.forward_gain, 0.35);
    EXPECT_DOUBLE_EQ(p.lateral_gain, 0.7);
    EXPECT_DOUBLE_EQ(p.yaw_gain, 1.0);
    EXPECT_DOUBLE_EQ(p.orbit_yaw_ki, 30.0);
    EXPECT_DOUBLE_EQ(p.max_forward_accel_m_s2, 0.0);  // smoothing is off unless asked for
    EXPECT_DOUBLE_EQ(p.max_strafe_accel_m_s2, 0.0);
    EXPECT_DOUBLE_EQ(p.max_yaw_accel_deg_s2, 0.0);
}

TEST(Gains, FollowUsesTheParameterisedGains) {
    FollowParams p;
    p.forward_gain = 0.5;
    p.lateral_gain = 0.5;
    p.yaw_gain = 0.5;
    p = sanitize(p);
    const VelocityCommand c = computeFollowCommand(1440.0, 7.0, camera(), p);  // half right, 1 m far
    EXPECT_NEAR(c.forward_m_s, 0.5, 1e-9);
    EXPECT_NEAR(c.right_m_s, 0.5 * 0.5 * 0.45, 1e-9);
    EXPECT_NEAR(c.yawspeed_deg_s, 0.5 * 0.5 * 45.0, 1e-9);
}

TEST(Gains, SanitizeEnforcesCeilingsAndFloors) {
    FollowParams p;
    p.forward_gain = 99.0;
    p.lateral_gain = 99.0;
    p.yaw_gain = 99.0;
    p.orbit_yaw_ki = 1e9;
    p.max_forward_accel_m_s2 = 99.0;
    p.max_strafe_accel_m_s2 = 99.0;
    p.max_yaw_accel_deg_s2 = 1e9;
    const FollowParams hi = sanitize(p);
    EXPECT_DOUBLE_EQ(hi.forward_gain, kForwardGainCeiling);
    EXPECT_DOUBLE_EQ(hi.lateral_gain, kLateralGainCeiling);
    EXPECT_DOUBLE_EQ(hi.yaw_gain, kYawGainCeiling);
    EXPECT_DOUBLE_EQ(hi.orbit_yaw_ki, kOrbitYawIntegralGainCeiling);
    EXPECT_DOUBLE_EQ(hi.max_forward_accel_m_s2, kForwardAccelCeilingMps2);
    EXPECT_DOUBLE_EQ(hi.max_strafe_accel_m_s2, kStrafeAccelCeilingMps2);
    EXPECT_DOUBLE_EQ(hi.max_yaw_accel_deg_s2, kYawAccelCeilingDegS2);
    p.forward_gain = -1.0;
    p.yaw_gain = -1.0;
    p.max_forward_accel_m_s2 = -2.0;
    const FollowParams lo = sanitize(p);
    EXPECT_DOUBLE_EQ(lo.forward_gain, 0.0);
    EXPECT_DOUBLE_EQ(lo.yaw_gain, 0.0);
    EXPECT_DOUBLE_EQ(lo.max_forward_accel_m_s2, 0.0);
}

TEST(Gains, NonFiniteGainsFallBackToTheDefaults) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    FollowParams p;
    p.forward_gain = nan;
    p.yaw_gain = nan;
    p.orbit_yaw_ki = nan;
    const FollowParams s = sanitize(p);
    EXPECT_DOUBLE_EQ(s.forward_gain, 0.35);
    EXPECT_DOUBLE_EQ(s.yaw_gain, 1.0);
    EXPECT_DOUBLE_EQ(s.orbit_yaw_ki, 30.0);
}

TEST(Limiter, OffByDefaultPassesCommandsStraightThrough) {
    CommandLimiter l;
    const VelocityCommand c = l.apply({0.7, -0.45, 45.0}, kP, 0.1);
    EXPECT_DOUBLE_EQ(c.forward_m_s, 0.7);
    EXPECT_DOUBLE_EQ(c.right_m_s, -0.45);
    EXPECT_DOUBLE_EQ(c.yawspeed_deg_s, 45.0);
}

TEST(Limiter, EachComponentChangesByAtMostLimitTimesDt) {
    FollowParams p;
    p.max_forward_accel_m_s2 = 1.0;
    p.max_strafe_accel_m_s2 = 2.0;
    p.max_yaw_accel_deg_s2 = 100.0;
    p = sanitize(p);
    CommandLimiter l;
    VelocityCommand c = l.apply({0.7, 0.45, 45.0}, p, 0.1);
    EXPECT_NEAR(c.forward_m_s, 0.1, 1e-12);
    EXPECT_NEAR(c.right_m_s, 0.2, 1e-12);
    EXPECT_NEAR(c.yawspeed_deg_s, 10.0, 1e-12);
    for (int i = 0; i < 100; ++i) c = l.apply({0.7, 0.45, 45.0}, p, 0.1);
    EXPECT_NEAR(c.forward_m_s, 0.7, 1e-12);  // reaches the target and stops there, no overshoot
    EXPECT_NEAR(c.right_m_s, 0.45, 1e-12);
    EXPECT_NEAR(c.yawspeed_deg_s, 45.0, 1e-12);
}

TEST(Limiter, RampsDownAndAcrossZeroAtTheSameRate) {
    FollowParams p;
    p.max_forward_accel_m_s2 = 1.0;
    p = sanitize(p);
    CommandLimiter l;
    for (int i = 0; i < 20; ++i) (void)l.apply({0.5, 0.0, 0.0}, p, 0.1);
    VelocityCommand c = l.apply({-0.5, 0.0, 0.0}, p, 0.1);
    EXPECT_NEAR(c.forward_m_s, 0.4, 1e-12);
}

TEST(Limiter, ResetStartsFromRestAndBadDtPassesThrough) {
    FollowParams p;
    p.max_forward_accel_m_s2 = 1.0;
    p = sanitize(p);
    CommandLimiter l;
    for (int i = 0; i < 50; ++i) (void)l.apply({0.7, 0.0, 0.0}, p, 0.1);
    l.reset();
    EXPECT_NEAR(l.apply({0.7, 0.0, 0.0}, p, 0.1).forward_m_s, 0.1, 1e-12);
    EXPECT_DOUBLE_EQ(l.apply({0.7, 0.0, 0.0}, p, 0.0).forward_m_s, 0.7);
}

TEST(Smoothing, OffByDefaultAndSanitizeClampsIt) {
    EXPECT_DOUBLE_EQ(FollowParams{}.command_smoothing_s, 0.0);
    FollowParams p;
    p.command_smoothing_s = 99.0;
    EXPECT_DOUBLE_EQ(sanitize(p).command_smoothing_s, kCommandSmoothingCeilingS);
    p.command_smoothing_s = -1.0;
    EXPECT_DOUBLE_EQ(sanitize(p).command_smoothing_s, 0.0);
    p.command_smoothing_s = std::nan("");
    EXPECT_DOUBLE_EQ(sanitize(p).command_smoothing_s, kDefaultCommandSmoothingS);
}

TEST(Smoothing, LagsAFirstStepAndApproachesTheTargetWithoutOvershoot) {
    FollowParams p;
    p.command_smoothing_s = 0.5;
    p = sanitize(p);
    CommandLimiter l;
    VelocityCommand c = l.apply({0.7, 0.0, 0.0}, p, 0.1);
    EXPECT_GT(c.forward_m_s, 0.0);
    EXPECT_LT(c.forward_m_s, 0.7 * 0.5);  // alpha = 0.1 / 0.6, so well short of the demand
    double previous = c.forward_m_s;
    for (int i = 0; i < 100; ++i) {
        c = l.apply({0.7, 0.0, 0.0}, p, 0.1);
        EXPECT_GE(c.forward_m_s, previous);  // monotonic: no overshoot or ringing
        EXPECT_LE(c.forward_m_s, 0.7 + 1e-12);
        previous = c.forward_m_s;
    }
    EXPECT_NEAR(c.forward_m_s, 0.7, 1e-6);
}

TEST(Smoothing, NeverExceedsTheAccelerationLimitAndBoundsTheJerk) {
    FollowParams p;
    p.max_forward_accel_m_s2 = 0.3;
    p.max_yaw_accel_deg_s2 = 30.0;
    p.command_smoothing_s = 0.4;
    p = sanitize(p);
    constexpr double dt = 0.1;
    CommandLimiter l;
    VelocityCommand prev;
    double prev_accel = 0.0;
    double max_jerk = 0.0;
    for (int i = 0; i < 400; ++i) {
        // Square wave demand, the harshest input: full forward, then full reverse.
        const double sign = ((i / 50) % 2 == 0) ? 1.0 : -1.0;
        const VelocityCommand c = l.apply({0.7 * sign, 0.0, 45.0 * sign}, p, dt);
        const double accel = (c.forward_m_s - prev.forward_m_s) / dt;
        EXPECT_LE(std::abs(accel), 0.3 + 1e-9);
        EXPECT_LE(std::abs(c.yawspeed_deg_s - prev.yawspeed_deg_s) / dt, 30.0 + 1e-9);
        EXPECT_LE(std::abs(c.forward_m_s), 0.7 + 1e-12);
        max_jerk = std::max(max_jerk, std::abs(accel - prev_accel) / dt);
        prev_accel = accel;
        prev = c;
    }
    // Without the low-pass, acceleration steps by the full limit in one tick: 0.3 / 0.1 = 3 m/s^3.
    EXPECT_LT(max_jerk, 0.5 * 0.3 / dt);
}

TEST(Smoothing, ResetAndBadDtBehaveAsWithoutIt) {
    FollowParams p;
    p.command_smoothing_s = 0.5;
    p = sanitize(p);
    CommandLimiter l;
    for (int i = 0; i < 100; ++i) (void)l.apply({0.7, 0.0, 0.0}, p, 0.1);
    l.reset();
    EXPECT_LT(l.apply({0.7, 0.0, 0.0}, p, 0.1).forward_m_s, 0.7 * 0.5);  // starts from rest
    EXPECT_DOUBLE_EQ(l.apply({0.7, 0.0, 0.0}, p, 0.0).forward_m_s, 0.7);  // bad dt: pass through
}

// ---------- search around the last known direction ----------

namespace {
FollowParams searchParams(double search_s = 8.0, double sweep = 60.0, double rate = 20.0) {
    FollowParams p;
    p.last_known_search_s = search_s;
    p.last_known_sweep_deg = sweep;
    p.last_known_rate_deg_s = rate;
    return sanitize(p);
}
}  // namespace

TEST(LastKnown, OffByDefaultSoTheOriginalSpiralRuns) {
    const FollowParams p = sanitize(FollowParams{});
    EXPECT_DOUBLE_EQ(p.last_known_search_s, 0.0);
    EXPECT_FALSE(lastKnownSearchActive(0.0, p));
    EXPECT_FALSE(lastKnownSearchActive(5.0, p));
}

TEST(LastKnown, ActiveOnlyUntilItsTimeLimit) {
    const FollowParams p = searchParams(8.0);
    EXPECT_TRUE(lastKnownSearchActive(0.0, p));
    EXPECT_TRUE(lastKnownSearchActive(8.0, p));
    EXPECT_FALSE(lastKnownSearchActive(8.01, p));
}

TEST(LastKnown, HoldsPositionAndOnlyTurns) {
    const FollowParams p = searchParams();
    for (double t = 0.0; t < 20.0; t += 0.1) {
        const VelocityCommand c = computeLastKnownSearchCommand(t, 1.0, p);
        EXPECT_DOUBLE_EQ(c.forward_m_s, 0.0);
        EXPECT_DOUBLE_EQ(c.right_m_s, 0.0);
        EXPECT_DOUBLE_EQ(std::abs(c.yawspeed_deg_s), 20.0);
    }
}

TEST(LastKnown, FirstTurnsTowardTheSideThePersonWasLast) {
    const FollowParams p = searchParams();
    EXPECT_GT(computeLastKnownSearchCommand(0.0, 1.0, p).yawspeed_deg_s, 0.0);    // right
    EXPECT_LT(computeLastKnownSearchCommand(0.0, -1.0, p).yawspeed_deg_s, 0.0);   // left
}

TEST(LastKnown, SweepStaysWithinTheArcAndCoversBothSides) {
    const FollowParams p = searchParams(60.0, 60.0, 20.0);
    const double dt = 0.01;
    double angle = 0.0, lo = 0.0, hi = 0.0;
    for (double t = 0.0; t < 24.0; t += dt) {  // two full sweeps (12 s each)
        angle += computeLastKnownSearchCommand(t, 1.0, p).yawspeed_deg_s * dt;
        lo = std::min(lo, angle);
        hi = std::max(hi, angle);
    }
    EXPECT_NEAR(hi, 60.0, 0.5);
    EXPECT_NEAR(lo, -60.0, 0.5);
    EXPECT_NEAR(angle, 0.0, 0.5);  // back where it started after whole sweeps
}

TEST(LastKnown, SweepArcAndRateAreParameters) {
    const FollowParams p = searchParams(60.0, 30.0, 10.0);
    double angle = 0.0, hi = 0.0;
    for (double t = 0.0; t < 12.0; t += 0.01) {
        angle += computeLastKnownSearchCommand(t, 1.0, p).yawspeed_deg_s * 0.01;
        hi = std::max(hi, angle);
    }
    EXPECT_NEAR(hi, 30.0, 0.5);
    EXPECT_DOUBLE_EQ(std::abs(computeLastKnownSearchCommand(1.0, 1.0, p).yawspeed_deg_s), 10.0);
}

TEST(LastKnown, SanitizeBoundsTheSearchAndNeverExceedsTheYawLimit) {
    FollowParams p;
    p.last_known_search_s = 1e9;
    p.last_known_sweep_deg = 1e9;
    p.last_known_rate_deg_s = 1e9;
    const FollowParams s = sanitize(p);
    EXPECT_DOUBLE_EQ(s.last_known_search_s, kLastKnownSearchCeilingS);
    EXPECT_DOUBLE_EQ(s.last_known_sweep_deg, kLastKnownSweepCeilingDeg);
    EXPECT_LE(s.last_known_rate_deg_s, s.max_yaw_rate_deg_s);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    p.last_known_search_s = nan;
    p.last_known_rate_deg_s = -5.0;
    const FollowParams f = sanitize(p);
    EXPECT_DOUBLE_EQ(f.last_known_search_s, 0.0);
    EXPECT_DOUBLE_EQ(f.last_known_rate_deg_s, 0.0);
}

TEST(LastKnown, ZeroRateOrArcOrBadTimeCommandsNothing) {
    EXPECT_DOUBLE_EQ(computeLastKnownSearchCommand(1.0, 1.0, searchParams(8.0, 60.0, 0.0))
                         .yawspeed_deg_s, 0.0);
    EXPECT_DOUBLE_EQ(computeLastKnownSearchCommand(1.0, 1.0, searchParams(8.0, 0.0, 20.0))
                         .yawspeed_deg_s, 0.0);
    EXPECT_DOUBLE_EQ(computeLastKnownSearchCommand(-1.0, 1.0, searchParams()).yawspeed_deg_s, 0.0);
    EXPECT_DOUBLE_EQ(computeLastKnownSearchCommand(std::numeric_limits<double>::quiet_NaN(), 1.0,
                                                   searchParams()).yawspeed_deg_s, 0.0);
}
