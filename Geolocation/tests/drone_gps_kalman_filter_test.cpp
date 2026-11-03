#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

#include "geolocation/drone_gps_kalman_filter.hpp"

namespace {

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void testOriginAndThreeSigma() {
    DroneGpsKalmanFilter filter(2.0, 3.0, 0.25);
    const auto estimate = filter.update(34.7, -86.6, 200.0, 0.0);

    require(std::abs(estimate.enu_m[0]) < 1e-9, "first fix should be ENU origin");
    require(std::abs(estimate.enu_m[1]) < 1e-9, "first fix should be ENU origin");
    require(std::abs(estimate.enu_m[2]) < 1e-9, "first fix should be ENU origin");
    require(
        std::abs(estimate.horizontal_radius_3sigma_m - 6.0) < 1e-9,
        "horizontal radius should use the larger horizontal standard deviation");
    require(
        std::abs(estimate.vertical_3sigma_m - 9.0) < 1e-9,
        "vertical uncertainty should be reported separately");
}

void testEnuConversionAndFiltering() {
    DroneGpsKalmanFilter filter(1.0, 1.0, 0.0);
    filter.update(34.7, -86.6, 200.0, 0.0);
    const auto estimate = filter.update(34.70001, -86.59999, 202.0, 1.0);

    require(estimate.enu_m[0] > 0.0, "eastward longitude change should be positive ENU x");
    require(estimate.enu_m[1] > 0.0, "northward latitude change should be positive ENU y");
    require(estimate.enu_m[2] > 0.0, "increased altitude should be positive ENU z");
    require(
        estimate.enu_m[2] < 2.0,
        "Kalman update should smooth a new measurement when process noise is zero");
}

void testInvalidInputs() {
    bool rejected_bad_sigma = false;
    try {
        DroneGpsKalmanFilter filter(0.0, 1.0, 0.25);
    } catch (const std::invalid_argument&) {
        rejected_bad_sigma = true;
    }
    require(rejected_bad_sigma, "zero standard deviation must be rejected");

    DroneGpsKalmanFilter filter(1.0, 1.0, 0.25);
    bool rejected_bad_position = false;
    try {
        filter.update(91.0, -86.6, 200.0, 0.0);
    } catch (const std::invalid_argument&) {
        rejected_bad_position = true;
    }
    require(rejected_bad_position, "out-of-range latitude must be rejected");
}

}  // namespace

int main() {
    try {
        testOriginAndThreeSigma();
        testEnuConversionAndFiltering();
        testInvalidInputs();
    } catch (const std::exception& error) {
        std::cerr << "GPS filter test failed: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
    std::cout << "GPS filter tests passed\n";
    return EXIT_SUCCESS;
}
