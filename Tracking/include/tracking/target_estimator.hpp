#ifndef TRACKING_TARGET_ESTIMATOR_HPP
#define TRACKING_TARGET_ESTIMATOR_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>

#include "tracking/geodesy.hpp"

namespace tracking {

struct Quaternion {
    double w{};
    double x{};
    double y{};
    double z{};
};

struct CameraIntrinsics {
    double fx_px{};
    double fy_px{};
    double cx_px{};
    double cy_px{};
};

struct DetectionBox {
    double confidence{};
    double x_min_px{};
    double y_min_px{};
    double x_max_px{};
    double y_max_px{};
};

struct Estimate {
    GeodeticPosition position;
    std::array<double, 3> error_ecef_3sigma_m{};
    double range_m{};
};

inline std::array<double, 3> rotate(
    const Quaternion& quaternion,
    const std::array<double, 3>& vector) {
    const double norm = std::sqrt(
        quaternion.w * quaternion.w + quaternion.x * quaternion.x +
        quaternion.y * quaternion.y + quaternion.z * quaternion.z);
    if (!std::isfinite(norm) || norm <= 0.0) {
        throw std::invalid_argument("attitude quaternion must be finite and non-zero");
    }
    const double w = quaternion.w / norm;
    const double x = quaternion.x / norm;
    const double y = quaternion.y / norm;
    const double z = quaternion.z / norm;
    const double tx = 2.0 * (y * vector[2] - z * vector[1]);
    const double ty = 2.0 * (z * vector[0] - x * vector[2]);
    const double tz = 2.0 * (x * vector[1] - y * vector[0]);
    return {
        vector[0] + w * tx + (y * tz - z * ty),
        vector[1] + w * ty + (z * tx - x * tz),
        vector[2] + w * tz + (x * ty - y * tx)
    };
}

inline Quaternion eulerQuaternion(double roll, double pitch, double yaw) {
    const double cr = std::cos(roll * 0.5);
    const double sr = std::sin(roll * 0.5);
    const double cp = std::cos(pitch * 0.5);
    const double sp = std::sin(pitch * 0.5);
    const double cy = std::cos(yaw * 0.5);
    const double sy = std::sin(yaw * 0.5);
    return {
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy
    };
}

inline std::array<double, 3> cameraOpticalToBodyFrd(
    const std::array<double, 3>& optical) {
    return {optical[2], optical[0], optical[1]};
}

inline std::array<double, 3> nedToEnu(const std::array<double, 3>& ned) {
    return {ned[1], ned[0], -ned[2]};
}

inline std::array<double, 3> enuToEcef(
    const std::array<double, 3>& enu,
    const GeodeticPosition& origin) {
    const double latitude = degreesToRadians(origin.latitude_deg);
    const double longitude = degreesToRadians(origin.longitude_deg);
    return {
        -std::sin(longitude) * enu[0] -
            std::sin(latitude) * std::cos(longitude) * enu[1] +
            std::cos(latitude) * std::cos(longitude) * enu[2],
        std::cos(longitude) * enu[0] -
            std::sin(latitude) * std::sin(longitude) * enu[1] +
            std::cos(latitude) * std::sin(longitude) * enu[2],
        std::cos(latitude) * enu[1] + std::sin(latitude) * enu[2]
    };
}

inline Estimate estimateTarget(
    const DetectionBox& detection,
    const CameraIntrinsics& camera,
    const GeodeticPosition& drone_position,
    const std::array<double, 3>& drone_error_ecef_3sigma_m,
    const Quaternion& body_frd_to_ned,
    double human_height_m,
    double human_height_stddev_m,
    double pixel_stddev_px,
    double camera_roll_rad,
    double camera_pitch_rad,
    double camera_yaw_rad,
    double minimum_confidence) {
    validatePosition(drone_position);
    if (!std::isfinite(detection.confidence) ||
        detection.confidence < minimum_confidence || detection.confidence > 1.0 ||
        !std::isfinite(detection.x_min_px) || !std::isfinite(detection.y_min_px) ||
        !std::isfinite(detection.x_max_px) || !std::isfinite(detection.y_max_px) ||
        detection.x_max_px <= detection.x_min_px ||
        detection.y_max_px <= detection.y_min_px) {
        throw std::invalid_argument("detection is invalid or below the confidence threshold");
    }
    if (!std::isfinite(camera.fx_px) || camera.fx_px <= 0.0 ||
        !std::isfinite(camera.fy_px) || camera.fy_px <= 0.0 ||
        !std::isfinite(camera.cx_px) || !std::isfinite(camera.cy_px) ||
        !std::isfinite(human_height_m) || human_height_m <= 0.0 ||
        !std::isfinite(human_height_stddev_m) || human_height_stddev_m <= 0.0 ||
        !std::isfinite(pixel_stddev_px) || pixel_stddev_px < 0.0 ||
        !std::isfinite(camera_roll_rad) || !std::isfinite(camera_pitch_rad) ||
        !std::isfinite(camera_yaw_rad) || !std::isfinite(minimum_confidence) ||
        minimum_confidence < 0.0 || minimum_confidence > 1.0) {
        throw std::invalid_argument("camera and target parameters are invalid");
    }
    for (const double error : drone_error_ecef_3sigma_m) {
        if (!std::isfinite(error) || error < 0.0) {
            throw std::invalid_argument("drone position errors must be finite and non-negative");
        }
    }

    const double box_height = detection.y_max_px - detection.y_min_px;
    const double range = human_height_m * camera.fy_px / box_height;
    if (!std::isfinite(range) || range <= 0.0) {
        throw std::invalid_argument("target range is invalid");
    }

    const double center_x = (detection.x_min_px + detection.x_max_px) * 0.5;
    const double center_y = (detection.y_min_px + detection.y_max_px) * 0.5;
    const double ray_x = (center_x - camera.cx_px) / camera.fx_px;
    const double ray_y = (center_y - camera.cy_px) / camera.fy_px;
    const std::array<double, 3> optical{
        ray_x * range,
        ray_y * range,
        range
    };
    const auto camera_mount = eulerQuaternion(
        camera_roll_rad, camera_pitch_rad, camera_yaw_rad);
    const auto body = rotate(camera_mount, cameraOpticalToBodyFrd(optical));
    const auto relative_enu = nedToEnu(rotate(body_frd_to_ned, body));
    const auto origin_ecef = geodeticToEcef(drone_position);
    const auto relative_ecef = enuToEcef(relative_enu, drone_position);
    const std::array<double, 3> target_ecef{
        origin_ecef[0] + relative_ecef[0],
        origin_ecef[1] + relative_ecef[1],
        origin_ecef[2] + relative_ecef[2]
    };

    const double sigma_box_height_px = std::sqrt(2.0) * pixel_stddev_px;
    const double sigma_range = std::hypot(
        range * human_height_stddev_m / human_height_m,
        range * sigma_box_height_px / box_height);
    if (!std::isfinite(sigma_range)) {
        throw std::invalid_argument("target range uncertainty is not finite");
    }
    const double range_variance = sigma_range * sigma_range;
    const std::array<std::array<double, 3>, 3> optical_covariance{{
        {std::pow(range * pixel_stddev_px / camera.fx_px, 2.0) +
             ray_x * ray_x * range_variance,
         ray_x * ray_y * range_variance,
         ray_x * range_variance},
        {ray_x * ray_y * range_variance,
         std::pow(range * pixel_stddev_px / camera.fy_px, 2.0) +
             ray_y * ray_y * range_variance,
         ray_y * range_variance},
        {ray_x * range_variance, ray_y * range_variance, range_variance}
    }};
    const std::array<std::array<double, 3>, 3> camera_basis{{
        {0.0, 1.0, 0.0},
        {0.0, 0.0, 1.0},
        {1.0, 0.0, 0.0}
    }};
    std::array<double, 3> error_ecef{};
    std::array<std::array<double, 3>, 3> covariance_ecef{};
    std::array<std::array<double, 3>, 3> ecef_basis{};
    for (std::size_t optical_axis = 0; optical_axis < 3; ++optical_axis) {
        std::array<double, 3> optical_basis{
            camera_basis[optical_axis][0],
            camera_basis[optical_axis][1],
            camera_basis[optical_axis][2]
        };
        const auto body_basis = rotate(camera_mount, optical_basis);
        const auto ned_basis = rotate(body_frd_to_ned, body_basis);
        const auto enu_basis = nedToEnu(ned_basis);
        ecef_basis[optical_axis] = enuToEcef(enu_basis, drone_position);
    }
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t column = 0; column < 3; ++column) {
            for (std::size_t first_axis = 0; first_axis < 3; ++first_axis) {
                for (std::size_t second_axis = 0; second_axis < 3; ++second_axis) {
                    covariance_ecef[row][column] +=
                        ecef_basis[first_axis][row] *
                        optical_covariance[first_axis][second_axis] *
                        ecef_basis[second_axis][column];
                }
            }
        }
    }
    for (std::size_t ecef_axis = 0; ecef_axis < 3; ++ecef_axis) {
        const double drone_sigma = drone_error_ecef_3sigma_m[ecef_axis] / 3.0;
        covariance_ecef[ecef_axis][ecef_axis] += drone_sigma * drone_sigma;
        error_ecef[ecef_axis] = 3.0 *
            std::sqrt(std::max(0.0, covariance_ecef[ecef_axis][ecef_axis]));
    }

    return {ecefToGeodetic(target_ecef), error_ecef, range};
}

}  // namespace tracking

#endif
