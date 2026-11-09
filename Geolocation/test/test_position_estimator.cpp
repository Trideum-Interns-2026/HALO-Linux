#include <cmath>
#include <stdexcept>

#include <gtest/gtest.h>

#include "geolocation/position_estimator.hpp"

TEST(PositionEstimator, FiltersGpsAndRotatesErrorsIntoEcef) {
    geolocation::PositionEstimator estimator(2.0, 3.0, 0.0);
    const auto first = estimator.update({34.7, -86.6, 200.0}, {4.0, 4.0, 9.0}, 0.0);
    EXPECT_NEAR(first.position.latitude_deg, 34.7, 1e-10);
    EXPECT_NEAR(first.position.longitude_deg, -86.6, 1e-10);
    EXPECT_NEAR(first.position.altitude_m, 200.0, 1e-5);
    EXPECT_GT(first.error_ecef_3sigma_m[0], 0.0);
    EXPECT_GT(first.error_ecef_3sigma_m[1], 0.0);
    EXPECT_GT(first.error_ecef_3sigma_m[2], 0.0);

    const auto next = estimator.update({34.70001, -86.59999, 202.0}, {4.0, 4.0, 9.0}, 1.0);
    EXPECT_GT(next.position.latitude_deg, first.position.latitude_deg);
    EXPECT_GT(next.position.longitude_deg, first.position.longitude_deg);
    EXPECT_LT(next.position.altitude_m, 202.0);
}

TEST(PositionEstimator, UsesConfiguredVariancesWhenGpsCovarianceIsUnknown) {
    geolocation::PositionEstimator estimator(2.0, 3.0, 0.0);
    const auto estimate = estimator.update({0.0, 0.0, 0.0}, {}, 0.0);
    EXPECT_NEAR(estimate.error_ecef_3sigma_m[0], 9.0, 1e-9);
    EXPECT_NEAR(estimate.error_ecef_3sigma_m[1], 6.0, 1e-9);
    EXPECT_NEAR(estimate.error_ecef_3sigma_m[2], 6.0, 1e-9);
}

TEST(PositionEstimator, RejectsInvalidInputs) {
    geolocation::PositionEstimator estimator(1.0, 1.0, 0.1);
    EXPECT_THROW(estimator.update({91.0, 0.0, 0.0}, {}, 0.0), std::invalid_argument);
    EXPECT_THROW(estimator.update({0.0, 0.0, 0.0}, {}, -1.0), std::invalid_argument);
}

TEST(CoordinateConversions, RoundTripsWgs84Position) {
    const geolocation::GeodeticPosition original{34.7, -86.6, 200.0};
    const auto converted = geolocation::ecefToGeodetic(
        geolocation::geodeticToEcef(original));
    EXPECT_NEAR(converted.latitude_deg, original.latitude_deg, 1e-9);
    EXPECT_NEAR(converted.longitude_deg, original.longitude_deg, 1e-9);
    EXPECT_NEAR(converted.altitude_m, original.altitude_m, 1e-4);
}
