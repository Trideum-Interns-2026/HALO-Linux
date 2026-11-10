// Turns a tracked person position (LLA), the drone position (LLA) and the drone heading into the
// pixel column and range the follow controller expects. No ROS and no MAVSDK in here, so it is
// unit-testable without either.
#pragma once

#include <cmath>
#include <optional>
#include <utility>

#include "movement/follow_logic.hpp"

namespace movement {

constexpr double kMaxSynthBearingDeg = 89.0;  // tan() stays finite
constexpr double kMinBearingRangeM = 0.5;     // closer than this: no meaningful bearing

// WGS84 ellipsoid.
constexpr double kWgs84SemiMajorM = 6378137.0;
constexpr double kWgs84EccSquared = 6.69437999014e-3;

struct GeoPoint {
    double latitude_deg{0.0};
    double longitude_deg{0.0};
    double altitude_m{0.0};
};

struct TargetSample {
    double x_px{0.0};
    std::optional<double> distance_m;  // nullopt = no range
};

// Degrees to (-180, 180]. Non-finite input stays non-finite.
[[nodiscard]] inline double wrap180(double deg) {
    double w = std::fmod(deg, 360.0);
    if (w <= -180.0) {
        w += 360.0;
    } else if (w > 180.0) {
        w -= 360.0;
    }
    return w;
}

// (north_m, east_m) of `person` seen from `drone`. WGS84 meridian and prime-vertical radii at the
// mean latitude; longitude difference wrapped to [-180, 180]. Empty if any input is non-finite or
// latitude is outside [-90, 90] or longitude outside [-180, 180].
[[nodiscard]] inline std::optional<std::pair<double, double>> localOffsetNorthEast(
    const GeoPoint& drone, const GeoPoint& person) {
    const auto valid = [](const GeoPoint& g) {
        return isFinite(g.latitude_deg) && isFinite(g.longitude_deg) && isFinite(g.altitude_m) &&
               g.latitude_deg >= -90.0 && g.latitude_deg <= 90.0 && g.longitude_deg >= -180.0 &&
               g.longitude_deg <= 180.0;
    };
    if (!valid(drone) || !valid(person)) {
        return std::nullopt;
    }
    const double deg_to_rad = kPi / 180.0;
    const double mean_lat = 0.5 * (drone.latitude_deg + person.latitude_deg) * deg_to_rad;
    const double sin_lat = std::sin(mean_lat);
    const double w = 1.0 - kWgs84EccSquared * sin_lat * sin_lat;
    const double sqrt_w = std::sqrt(w);
    const double meridian_radius = kWgs84SemiMajorM * (1.0 - kWgs84EccSquared) / (w * sqrt_w);
    const double prime_vertical_radius = kWgs84SemiMajorM / sqrt_w;

    const double dlat = (person.latitude_deg - drone.latitude_deg) * deg_to_rad;
    const double dlon = wrap180(person.longitude_deg - drone.longitude_deg) * deg_to_rad;
    const double north = dlat * meridian_radius;
    const double east = dlon * prime_vertical_radius * std::cos(mean_lat);
    if (!isFinite(north) || !isFinite(east)) {
        return std::nullopt;
    }
    return std::make_pair(north, east);
}

// Compass bearing of an offset, degrees clockwise from north.
[[nodiscard]] inline double worldBearingDeg(double north_m, double east_m) {
    return std::atan2(east_m, north_m) * 180.0 / kPi;
}

// Bearing relative to the heading, (-180, 180], + = to the right of the nose.
[[nodiscard]] inline double relativeBearingDeg(double world_deg, double heading_deg) {
    return wrap180(world_deg - heading_deg);
}

// Pinhole column for a relative bearing: cx + fx * tan(clamp(rel, -89, 89) deg).
[[nodiscard]] inline double targetPixelX(double relative_bearing_deg,
                                         const CameraGeometry& camera) {
    const double rel =
        clamp(relative_bearing_deg, -kMaxSynthBearingDeg, kMaxSynthBearingDeg);
    return camera.center_x_px + camera.focal_x_px * std::tan(rel * kPi / 180.0);
}

// Empty if the heading is missing or non-finite, the camera is not usable, or the offset is
// invalid. The distance is the horizontal ground range (altitude ignored) and is given ONLY while
// x_px is within [0, width_px]; otherwise it is empty (target behind or beside: yaw and strafe
// only, never forward). Closer than kMinBearingRangeM the bearing means nothing: x_px is the
// principal point and there is no range (hover).
[[nodiscard]] inline std::optional<TargetSample> synthesizeTarget(
    const GeoPoint& drone, const GeoPoint& person, std::optional<double> heading_deg,
    const CameraGeometry& camera) {
    if (!heading_deg || !isFinite(*heading_deg) || !isUsable(camera)) {
        return std::nullopt;
    }
    const auto offset = localOffsetNorthEast(drone, person);
    if (!offset) {
        return std::nullopt;
    }
    const double range = std::hypot(offset->first, offset->second);
    TargetSample sample;
    if (!(range >= kMinBearingRangeM)) {
        sample.x_px = camera.center_x_px;
        return sample;
    }
    const double world = worldBearingDeg(offset->first, offset->second);
    sample.x_px = targetPixelX(relativeBearingDeg(world, *heading_deg), camera);
    if (sample.x_px >= 0.0 && sample.x_px <= camera.width_px) {
        sample.distance_m = range;
    }
    return sample;
}

// True = REJECT. max_m == 0 turns the check off. Otherwise the Euclidean norm of the 3-sigma
// triplet is compared with max_m (exactly at the limit is accepted). Non-finite or negative
// components are always rejected, even when max_m == 0; so is a non-finite or negative max_m.
[[nodiscard]] inline bool rejectTargetError(double error_x_m, double error_y_m, double error_z_m,
                                            double max_m) {
    for (const double e : {error_x_m, error_y_m, error_z_m}) {
        if (!isFinite(e) || e < 0.0) {
            return true;
        }
    }
    if (max_m == 0.0) {
        return false;
    }
    if (!isFinite(max_m) || max_m < 0.0) {
        return true;
    }
    const double norm = std::sqrt(error_x_m * error_x_m + error_y_m * error_y_m +
                                  error_z_m * error_z_m);
    return norm > max_m;
}

}  // namespace movement
