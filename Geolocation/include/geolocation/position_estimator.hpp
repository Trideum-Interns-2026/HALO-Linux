#ifndef GEOLOCATION_POSITION_ESTIMATOR_HPP
#define GEOLOCATION_POSITION_ESTIMATOR_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace geolocation {

constexpr double kPi = 3.14159265358979323846;
constexpr double kWgs84SemiMajorAxisM = 6378137.0;
constexpr double kWgs84EccentricitySquared = 6.6943799901413165e-3;

struct GeodeticPosition {
    double latitude_deg{};
    double longitude_deg{};
    double altitude_m{};
};

struct Estimate {
    GeodeticPosition position;
    std::array<double, 3> error_ecef_3sigma_m{};
};

inline double degreesToRadians(double degrees) {
    return degrees * kPi / 180.0;
}

inline void validatePosition(const GeodeticPosition& position) {
    if (!std::isfinite(position.latitude_deg) || position.latitude_deg < -90.0 ||
        position.latitude_deg > 90.0 || !std::isfinite(position.longitude_deg) ||
        position.longitude_deg < -180.0 || position.longitude_deg > 180.0 ||
        !std::isfinite(position.altitude_m)) {
        throw std::invalid_argument("position must contain finite WGS84 coordinates");
    }
}

inline std::array<double, 3> geodeticToEcef(const GeodeticPosition& position) {
    validatePosition(position);
    const double latitude = degreesToRadians(position.latitude_deg);
    const double longitude = degreesToRadians(position.longitude_deg);
    const double sin_latitude = std::sin(latitude);
    const double cos_latitude = std::cos(latitude);
    const double prime_vertical_radius = kWgs84SemiMajorAxisM /
        std::sqrt(1.0 - kWgs84EccentricitySquared * sin_latitude * sin_latitude);
    return {
        (prime_vertical_radius + position.altitude_m) * cos_latitude * std::cos(longitude),
        (prime_vertical_radius + position.altitude_m) * cos_latitude * std::sin(longitude),
        (prime_vertical_radius * (1.0 - kWgs84EccentricitySquared) +
            position.altitude_m) * sin_latitude
    };
}

inline GeodeticPosition ecefToGeodetic(const std::array<double, 3>& ecef) {
    if (!std::isfinite(ecef[0]) || !std::isfinite(ecef[1]) || !std::isfinite(ecef[2])) {
        throw std::invalid_argument("ECEF position must be finite");
    }

    const double longitude = std::atan2(ecef[1], ecef[0]);
    const double horizontal = std::hypot(ecef[0], ecef[1]);
    if (horizontal < 1e-8) {
        const double semi_minor_axis = kWgs84SemiMajorAxisM *
            std::sqrt(1.0 - kWgs84EccentricitySquared);
        return {
            std::copysign(90.0, ecef[2]),
            0.0,
            std::abs(ecef[2]) - semi_minor_axis
        };
    }
    double latitude = std::atan2(
        ecef[2],
        horizontal * (1.0 - kWgs84EccentricitySquared));
    double altitude = 0.0;
    for (int iteration = 0; iteration < 8; ++iteration) {
        const double sin_latitude = std::sin(latitude);
        const double radius = kWgs84SemiMajorAxisM /
            std::sqrt(1.0 - kWgs84EccentricitySquared * sin_latitude * sin_latitude);
        altitude = horizontal / std::cos(latitude) - radius;
        const double next_latitude = std::atan2(
            ecef[2],
            horizontal * (1.0 - kWgs84EccentricitySquared *
                radius / (radius + altitude)));
        if (std::abs(next_latitude - latitude) < 1e-13) {
            latitude = next_latitude;
            break;
        }
        latitude = next_latitude;
    }

    return {latitude * 180.0 / kPi, longitude * 180.0 / kPi, altitude};
}

class PositionEstimator {
public:
    PositionEstimator(
        double fallback_horizontal_stddev_m,
        double fallback_vertical_stddev_m,
        double process_noise_variance_m2_per_s)
        : fallback_variance_{
              squarePositive(fallback_horizontal_stddev_m),
              squarePositive(fallback_horizontal_stddev_m),
              squarePositive(fallback_vertical_stddev_m)},
          process_noise_variance_(process_noise_variance_m2_per_s) {
        if (!std::isfinite(process_noise_variance_) || process_noise_variance_ < 0.0) {
            throw std::invalid_argument("process noise must be finite and non-negative");
        }
    }

    Estimate update(
        const GeodeticPosition& measurement,
        const std::array<double, 3>& measurement_variance_enu_m2,
        double elapsed_seconds) {
        validatePosition(measurement);
        if (!std::isfinite(elapsed_seconds) || elapsed_seconds < 0.0) {
            throw std::invalid_argument("elapsed time must be finite and non-negative");
        }
        for (const double variance : measurement_variance_enu_m2) {
            if (!std::isfinite(variance) || variance < 0.0) {
                throw std::invalid_argument("measurement variances must be finite and non-negative");
            }
        }

        if (!initialized_) {
            origin_ = measurement;
            position_enu_m_.fill(0.0);
            for (std::size_t axis = 0; axis < 3; ++axis) {
                variance_enu_m2_[axis] = effectiveVariance(
                    measurement_variance_enu_m2[axis], fallback_variance_[axis]);
            }
            initialized_ = true;
            return makeEstimate();
        }

        const auto measurement_enu = toEnu(measurement);
        for (std::size_t axis = 0; axis < 3; ++axis) {
            const double observation_variance = effectiveVariance(
                measurement_variance_enu_m2[axis], fallback_variance_[axis]);
            const double predicted_variance =
                variance_enu_m2_[axis] + process_noise_variance_ * elapsed_seconds;
            if (!std::isfinite(predicted_variance) ||
                !std::isfinite(observation_variance) ||
                predicted_variance + observation_variance <= 0.0) {
                throw std::invalid_argument("GPS filter variance is not finite and positive");
            }
            const double gain = predicted_variance /
                (predicted_variance + observation_variance);
            position_enu_m_[axis] += gain *
                (measurement_enu[axis] - position_enu_m_[axis]);
            variance_enu_m2_[axis] = predicted_variance * (1.0 - gain);
        }
        return makeEstimate();
    }

private:
    static double squarePositive(double value) {
        if (!std::isfinite(value) || value <= 0.0) {
            throw std::invalid_argument("fallback standard deviations must be positive");
        }
        const double variance = value * value;
        if (!std::isfinite(variance)) {
            throw std::invalid_argument("fallback standard deviation is too large");
        }
        return variance;
    }

    static double effectiveVariance(double reported, double fallback) {
        return reported > 0.0 ? reported : fallback;
    }

    std::array<double, 3> toEnu(const GeodeticPosition& position) const {
        const auto ecef = geodeticToEcef(position);
        const auto origin_ecef = geodeticToEcef(origin_);
        const std::array<double, 3> delta{
            ecef[0] - origin_ecef[0],
            ecef[1] - origin_ecef[1],
            ecef[2] - origin_ecef[2]
        };
        const double latitude = degreesToRadians(origin_.latitude_deg);
        const double longitude = degreesToRadians(origin_.longitude_deg);
        return {
            -std::sin(longitude) * delta[0] + std::cos(longitude) * delta[1],
            -std::sin(latitude) * std::cos(longitude) * delta[0] -
                std::sin(latitude) * std::sin(longitude) * delta[1] +
                std::cos(latitude) * delta[2],
            std::cos(latitude) * std::cos(longitude) * delta[0] +
                std::cos(latitude) * std::sin(longitude) * delta[1] +
                std::sin(latitude) * delta[2]
        };
    }

    Estimate makeEstimate() const {
        const double latitude = degreesToRadians(origin_.latitude_deg);
        const double longitude = degreesToRadians(origin_.longitude_deg);
        const double east = position_enu_m_[0];
        const double north = position_enu_m_[1];
        const double up = position_enu_m_[2];
        const auto origin_ecef = geodeticToEcef(origin_);
        const std::array<double, 3> estimate_ecef{
            origin_ecef[0] - std::sin(longitude) * east -
                std::sin(latitude) * std::cos(longitude) * north +
                std::cos(latitude) * std::cos(longitude) * up,
            origin_ecef[1] + std::cos(longitude) * east -
                std::sin(latitude) * std::sin(longitude) * north +
                std::cos(latitude) * std::sin(longitude) * up,
            origin_ecef[2] + std::cos(latitude) * north +
                std::sin(latitude) * up
        };

        const std::array<std::array<double, 3>, 3> enu_basis_in_ecef{{
            {-std::sin(longitude), std::cos(longitude), 0.0},
            {-std::sin(latitude) * std::cos(longitude),
             -std::sin(latitude) * std::sin(longitude), std::cos(latitude)},
            {std::cos(latitude) * std::cos(longitude),
             std::cos(latitude) * std::sin(longitude), std::sin(latitude)}
        }};
        std::array<double, 3> errors{};
        for (std::size_t ecef_axis = 0; ecef_axis < 3; ++ecef_axis) {
            double variance = 0.0;
            for (std::size_t enu_axis = 0; enu_axis < 3; ++enu_axis) {
                const double basis = enu_basis_in_ecef[enu_axis][ecef_axis];
                variance += basis * basis * variance_enu_m2_[enu_axis];
            }
            errors[ecef_axis] = 3.0 * std::sqrt(std::max(0.0, variance));
        }
        return {ecefToGeodetic(estimate_ecef), errors};
    }

    GeodeticPosition origin_{};
    std::array<double, 3> position_enu_m_{};
    std::array<double, 3> variance_enu_m2_{};
    std::array<double, 3> fallback_variance_{};
    double process_noise_variance_;
    bool initialized_{false};
};

}  // namespace geolocation

#endif
