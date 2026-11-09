#ifndef TRACKING_GEODESY_HPP
#define TRACKING_GEODESY_HPP

#include <array>
#include <cmath>
#include <stdexcept>

namespace tracking {

constexpr double kPi = 3.14159265358979323846;
constexpr double kWgs84SemiMajorAxisM = 6378137.0;
constexpr double kWgs84EccentricitySquared = 6.6943799901413165e-3;

struct GeodeticPosition {
    double latitude_deg{};
    double longitude_deg{};
    double altitude_m{};
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
    const double radius = kWgs84SemiMajorAxisM /
        std::sqrt(1.0 - kWgs84EccentricitySquared * sin_latitude * sin_latitude);
    return {
        (radius + position.altitude_m) * cos_latitude * std::cos(longitude),
        (radius + position.altitude_m) * cos_latitude * std::sin(longitude),
        (radius * (1.0 - kWgs84EccentricitySquared) + position.altitude_m) * sin_latitude
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
        ecef[2], horizontal * (1.0 - kWgs84EccentricitySquared));
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

}  // namespace tracking

#endif
