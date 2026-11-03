#ifndef GROUNDSTATION_DRONE_GPS_KALMAN_FILTER_HPP
#define GROUNDSTATION_DRONE_GPS_KALMAN_FILTER_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

class DroneGpsKalmanFilter {
public:
    struct Estimate {
        std::array<double, 3> enu_m{};
        std::array<double, 3> ecef_m{};
        double latitude_deg{0.0};
        double longitude_deg{0.0};
        double altitude_m{0.0};
        std::array<double, 9> covariance_enu{};
        double horizontal_radius_3sigma_m{0.0};
        double vertical_3sigma_m{0.0};
    };

    DroneGpsKalmanFilter(
        double horizontal_stddev_m,
        double vertical_stddev_m,
        double process_noise_variance_m2_per_s)
        : horizontal_variance_(squarePositive(horizontal_stddev_m)),
          vertical_variance_(squarePositive(vertical_stddev_m)),
          process_noise_variance_(process_noise_variance_m2_per_s) {
        if (!std::isfinite(process_noise_variance_) || process_noise_variance_ < 0.0) {
            throw std::invalid_argument("GPS filter process noise must be finite and non-negative");
        }
    }

    Estimate update(
        double latitude_deg,
        double longitude_deg,
        double altitude_m,
        double elapsed_seconds) {
        validatePosition(latitude_deg, longitude_deg, altitude_m);
        if (!std::isfinite(elapsed_seconds) || elapsed_seconds < 0.0) {
            throw std::invalid_argument("GPS filter elapsed time must be finite and non-negative");
        }

        const bool first_fix = !initialized_;
        if (first_fix) {
            origin_latitude_deg_ = latitude_deg;
            origin_longitude_deg_ = longitude_deg;
            origin_altitude_m_ = altitude_m;
            initializeAxis(east_, 0.0, horizontal_variance_);
            initializeAxis(north_, 0.0, horizontal_variance_);
            initializeAxis(up_, 0.0, vertical_variance_);
            initialized_ = true;
        }

        const auto measurement = toEnu(latitude_deg, longitude_deg, altitude_m);
        if (!first_fix) {
            updateAxis(east_, measurement[0], horizontal_variance_, elapsed_seconds);
            updateAxis(north_, measurement[1], horizontal_variance_, elapsed_seconds);
            updateAxis(up_, measurement[2], vertical_variance_, elapsed_seconds);
        }

        const double east_stddev = std::sqrt(east_.variance);
        const double north_stddev = std::sqrt(north_.variance);
        const double vertical_stddev = std::sqrt(up_.variance);

        const std::array<double, 3> enu{east_.position, north_.position, up_.position};
        const std::array<double, 9> covariance_enu = {
            east_stddev * east_stddev, 0.0, 0.0,
            0.0, north_stddev * north_stddev, 0.0,
            0.0, 0.0, vertical_stddev * vertical_stddev
        };

        const std::array<double, 3> ecef = enuToEcef(enu);

        return {
            enu,
            ecef,
            origin_latitude_deg_ + enu[1] / metersPerDegreeLatitude(),
            origin_longitude_deg_ + enu[0] /
                (metersPerDegreeLatitude() * std::cos(degToRad(origin_latitude_deg_))),
            origin_altitude_m_ + enu[2],
            covariance_enu,
            3.0 * std::max(east_stddev, north_stddev),
            3.0 * vertical_stddev
        };
    }

    static void validatePosition(double latitude_deg, double longitude_deg, double altitude_m) {
        if (!std::isfinite(latitude_deg) || latitude_deg < -90.0 || latitude_deg > 90.0 ||
            !std::isfinite(longitude_deg) || longitude_deg < -180.0 || longitude_deg > 180.0 ||
            !std::isfinite(altitude_m)) {
            throw std::invalid_argument("GPS position must contain finite WGS84 coordinates and altitude");
        }
        if (std::abs(std::cos(degToRad(latitude_deg))) < 1e-6) {
            throw std::invalid_argument("Local ENU projection is undefined near the poles");
        }
    }

private:
    struct Axis {
        double position{0.0};
        double variance{0.0};
    };

    static double squarePositive(double value) {
        if (!std::isfinite(value) || value <= 0.0) {
            throw std::invalid_argument("GPS standard deviations must be finite and greater than zero");
        }
        const double variance = value * value;
        if (!std::isfinite(variance)) {
            throw std::invalid_argument("GPS standard deviation is too large");
        }
        return variance;
    }

    static double degToRad(double degrees) {
        return degrees * 3.14159265358979323846 / 180.0;
    }

    static double metersPerDegreeLatitude() {
        return 111319.49079327358;
    }

    static void initializeAxis(Axis& axis, double position, double measurement_variance) {
        axis.position = position;
        axis.variance = measurement_variance;
    }

    void updateAxis(
        Axis& axis,
        double measurement,
        double measurement_variance,
        double elapsed_seconds) const {
        axis.variance += process_noise_variance_ * elapsed_seconds;
        const double gain = axis.variance / (axis.variance + measurement_variance);
        axis.position += gain * (measurement - axis.position);
        axis.variance *= 1.0 - gain;
    }

    std::array<double, 3> toEnu(
        double latitude_deg,
        double longitude_deg,
        double altitude_m) const {
        const double latitude_scale = metersPerDegreeLatitude();
        const double longitude_scale =
            latitude_scale * std::cos(degToRad(origin_latitude_deg_));
        return {
            (longitude_deg - origin_longitude_deg_) * longitude_scale,
            (latitude_deg - origin_latitude_deg_) * latitude_scale,
            altitude_m - origin_altitude_m_
        };
    }

    std::array<double, 3> enuToEcef(const std::array<double, 3>& enu) const {
        const double x = enu[0];
        const double y = enu[1];
        const double z = enu[2];
        return {x, y, z};
    }

    double horizontal_variance_;
    double vertical_variance_;
    double process_noise_variance_;
    double origin_latitude_deg_{0.0};
    double origin_longitude_deg_{0.0};
    double origin_altitude_m_{0.0};
    Axis east_;
    Axis north_;
    Axis up_;
    bool initialized_{false};
};

#endif
