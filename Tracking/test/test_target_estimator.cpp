#include <cmath>
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
