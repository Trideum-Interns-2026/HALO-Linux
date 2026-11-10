#include <cmath>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "tracking/target_estimator.hpp"

TEST(TargetEstimator, ProjectsCenteredPersonUsingPx4NedAttitude) {
    const tracking::DetectionBox detection{0.9, 270.0, 190.0, 370.0, 290.0};
    const tracking::CameraIntrinsics camera{500.0, 500.0, 320.0, 240.0};
    const tracking::GeodeticPosition drone{0.0, 0.0, 100.0};
    const std::array<double, 3> drone_error{0.3, 0.3, 0.3};
    const tracking::Quaternion identity{1.0, 0.0, 0.0, 0.0};

    const auto estimate = tracking::estimateTarget(
        detection, camera, drone, drone_error, identity,
        1.75, 0.15, 2.0, 0.0, 0.0, 0.0, 0.5);

    EXPECT_NEAR(estimate.range_m, 8.75, 1e-12);
    EXPECT_GT(estimate.position.latitude_deg, drone.latitude_deg);
    EXPECT_NEAR(estimate.position.longitude_deg, drone.longitude_deg, 1e-8);
    EXPECT_NEAR(estimate.position.altitude_m, drone.altitude_m, 1e-4);
    EXPECT_GT(estimate.error_ecef_3sigma_m[0], 0.0);
    EXPECT_GT(estimate.error_ecef_3sigma_m[1], 0.0);
    EXPECT_GT(estimate.error_ecef_3sigma_m[2], 0.0);
}

TEST(TargetEstimator, RejectsInvalidAndLowConfidenceBoxes) {
    const tracking::CameraIntrinsics camera{500.0, 500.0, 320.0, 240.0};
    const tracking::GeodeticPosition drone{34.7, -86.6, 200.0};
    const std::array<double, 3> error{1.0, 1.0, 1.0};
    const tracking::Quaternion identity{1.0, 0.0, 0.0, 0.0};
    EXPECT_THROW(
        tracking::estimateTarget(
            {0.4, 270.0, 190.0, 370.0, 290.0}, camera, drone, error,
            identity, 1.75, 0.15, 2.0, 0.0, 0.0, 0.0, 0.5),
        std::invalid_argument);
    EXPECT_THROW(
        tracking::estimateTarget(
            {0.9, 270.0, 290.0, 370.0, 190.0}, camera, drone, error,
            identity, 1.75, 0.15, 2.0, 0.0, 0.0, 0.0, 0.5),
        std::invalid_argument);
}

TEST(TargetEstimator, RotatesPx4NedVectorToEast) {
    const tracking::Quaternion yaw_ninety_degrees{
        std::sqrt(0.5), 0.0, 0.0, std::sqrt(0.5)};
    const auto east = tracking::nedToEnu(
        tracking::rotate(yaw_ninety_degrees, {1.0, 0.0, 0.0}));
    EXPECT_NEAR(east[0], 1.0, 1e-12);
    EXPECT_NEAR(east[1], 0.0, 1e-12);
}

struct EdgeCase {
    double y_min;
    double y_max;
    double height;
    double margin;
    bool clipped;
};

class FrameEdgeTest : public ::testing::TestWithParam<EdgeCase> {};

TEST_P(FrameEdgeTest, MatchesExpected) {
    const auto c = GetParam();
    EXPECT_EQ(
        tracking::boxTouchesVerticalFrameEdge(c.y_min, c.y_max, c.height, c.margin), c.clipped);
}

static const double kNaN = std::numeric_limits<double>::quiet_NaN();
static const double kInf = std::numeric_limits<double>::infinity();

INSTANTIATE_TEST_SUITE_P(
    TargetEstimator, FrameEdgeTest,
    ::testing::Values(
        EdgeCase{190.0, 290.0, 480.0, 2.0, false},   // mid-frame
        EdgeCase{2.0, 290.0, 480.0, 2.0, true},      // top exactly at margin
        EdgeCase{2.5, 290.0, 480.0, 2.0, false},     // top just inside
        EdgeCase{1.0, 290.0, 480.0, 2.0, true},      // top beyond margin
        EdgeCase{190.0, 478.0, 480.0, 2.0, true},    // bottom exactly at margin
        EdgeCase{190.0, 477.5, 480.0, 2.0, false},   // bottom just inside
        EdgeCase{190.0, 480.0, 480.0, 2.0, true},    // bottom at image edge
        EdgeCase{0.0, 290.0, 480.0, 0.0, true},      // margin 0, top at edge
        EdgeCase{190.0, 480.0, 480.0, 0.0, true},    // margin 0, bottom at edge
        EdgeCase{0.5, 479.5, 480.0, 0.0, false},     // margin 0, just inside
        EdgeCase{0.0, 480.0, 0.0, 2.0, false},       // unknown height 0
        EdgeCase{0.0, 480.0, -5.0, 2.0, false},      // negative height
        EdgeCase{0.0, 480.0, kNaN, 2.0, false},      // NaN height
        EdgeCase{0.0, 480.0, kInf, 2.0, false},      // infinite height
        EdgeCase{kNaN, 290.0, 480.0, 2.0, true},     // non-finite box
        EdgeCase{190.0, kNaN, 480.0, 2.0, true},
        EdgeCase{190.0, kInf, 480.0, 2.0, true},
        EdgeCase{-kInf, 290.0, 480.0, 2.0, true}));

TEST(TargetEstimator, SanitizesFrameEdgeMargin) {
    EXPECT_DOUBLE_EQ(tracking::sanitizeFrameEdgeMargin(2.0), 2.0);
    EXPECT_DOUBLE_EQ(tracking::sanitizeFrameEdgeMargin(0.0), 0.0);
    EXPECT_DOUBLE_EQ(tracking::sanitizeFrameEdgeMargin(-3.0), 0.0);
    EXPECT_DOUBLE_EQ(
        tracking::sanitizeFrameEdgeMargin(1e9), tracking::kMaxFrameEdgeMarginPx);
    EXPECT_DOUBLE_EQ(
        tracking::sanitizeFrameEdgeMargin(kNaN), tracking::kDefaultFrameEdgeMarginPx);
    EXPECT_DOUBLE_EQ(
        tracking::sanitizeFrameEdgeMargin(kInf), tracking::kDefaultFrameEdgeMarginPx);
}
