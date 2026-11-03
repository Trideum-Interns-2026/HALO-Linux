#ifndef TARGET_KALMAN_FILTER_HPP
#define TARGET_KALMAN_FILTER_HPP

#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <Eigen/Dense>

class TargetKalmanFilter {
public:
    struct Estimate {
        std::array<double, 3> local_enu_m{};
        std::array<double, 9> covariance_enu{};
        double latitude_deg{0.0};
        double longitude_deg{0.0};
        double altitude_m{0.0};
        double range_m{0.0};
        double confidence{0.0};
    };

    explicit TargetKalmanFilter(
        double human_height_m = 1.75,
        double human_height_stddev_m = 0.10,
        double process_noise_stddev_m = 0.50)
        : human_height_m_(human_height_m),
          human_height_stddev_m_(human_height_stddev_m),
          process_noise_variance_(squarePositive(process_noise_stddev_m)) {
        if (!std::isfinite(human_height_m_) || human_height_m_ <= 0.0) {
            throw std::invalid_argument("human_height_m must be finite and positive");
        }
        if (!std::isfinite(human_height_stddev_m_) || human_height_stddev_m_ <= 0.0) {
            throw std::invalid_argument("human_height_stddev_m must be finite and positive");
        }
        state_.setZero();
        covariance_ = Eigen::Matrix3d::Identity() * 10.0;
    }

    static double estimateRangeFromBoxHeight(
        double box_height_px,
        double focal_length_px,
        double human_height_m = 1.75) {
        if (!std::isfinite(box_height_px) || !std::isfinite(focal_length_px) ||
            box_height_px <= 0.0 || focal_length_px <= 0.0 ||
            !std::isfinite(human_height_m) || human_height_m <= 0.0) {
            return std::numeric_limits<double>::quiet_NaN();
        }
        return (human_height_m * focal_length_px) / box_height_px;
    }

    void predict(double dt) {
        if (!std::isfinite(dt) || dt < 0.0) {
            throw std::invalid_argument("prediction time must be finite and non-negative");
        }
        if (dt == 0.0) {
            return;
        }
        Eigen::Matrix3d transition = Eigen::Matrix3d::Identity();
        transition(0, 0) = 1.0;
        transition(1, 1) = 1.0;
        transition(2, 2) = 1.0;
        state_ = transition * state_;
        covariance_ = transition * covariance_ * transition.transpose() +
            Eigen::Matrix3d::Identity() * process_noise_variance_ * dt;
    }

    void updateFromImage(
        double image_u_px,
        double image_v_px,
        double box_height_px,
        double focal_length_px,
        double image_center_u_px,
        double image_center_v_px,
        double dt) {
        const double range_m = estimateRangeFromBoxHeight(box_height_px, focal_length_px, human_height_m_);
        if (!std::isfinite(range_m) || range_m <= 0.0) {
            throw std::invalid_argument("range estimate is invalid; box height and focal length must be finite and positive");
        }

        const double delta_u = image_u_px - image_center_u_px;
        const double delta_v = image_v_px - image_center_v_px;
        const double local_x = (delta_u / focal_length_px) * range_m;
        const double local_y = (delta_v / focal_length_px) * range_m;
        const double local_z = range_m;

        Eigen::Vector3d measurement;
        measurement << local_x, local_y, local_z;

        const double range_variance =
            std::pow((human_height_m_ / box_height_px) * 0.0, 2.0) +
            std::pow((focal_length_px / box_height_px) * human_height_stddev_m_, 2.0);
        const double sigma_x = std::sqrt(std::max(0.01, range_variance + std::pow(delta_u * 0.01, 2.0)));
        const double sigma_y = std::sqrt(std::max(0.01, range_variance + std::pow(delta_v * 0.01, 2.0)));
        const double sigma_z = std::sqrt(std::max(0.1, range_variance));

        Eigen::Matrix3d measurement_covariance = Eigen::Matrix3d::Zero();
        measurement_covariance(0, 0) = sigma_x * sigma_x;
        measurement_covariance(1, 1) = sigma_y * sigma_y;
        measurement_covariance(2, 2) = sigma_z * sigma_z;

        if (!initialized_) {
            state_ = measurement;
            covariance_ = measurement_covariance;
            initialized_ = true;
            return;
        }

        predict(dt);
        const Eigen::Matrix3d innovation_covariance = covariance_ + measurement_covariance;
        const Eigen::Matrix3d kalman_gain = covariance_ * innovation_covariance.inverse();
        const Eigen::Vector3d residual = measurement - state_;
        state_ += kalman_gain * residual;
        covariance_ = (Eigen::Matrix3d::Identity() - kalman_gain) * covariance_;
    }

    Estimate estimate(double origin_latitude_deg, double origin_longitude_deg, double origin_altitude_m) const {
        const double latitude_scale = 111319.49079327358;
        const double longitude_scale = latitude_scale * std::cos(degToRad(origin_latitude_deg));
        const double lat = origin_latitude_deg + state_(1) / latitude_scale;
        const double lon = origin_longitude_deg + state_(0) / longitude_scale;
        const double alt = origin_altitude_m + state_(2);

        std::array<double, 9> covariance_flat = {
            covariance_(0, 0), covariance_(0, 1), covariance_(0, 2),
            covariance_(1, 0), covariance_(1, 1), covariance_(1, 2),
            covariance_(2, 0), covariance_(2, 1), covariance_(2, 2)
        };

        return {
            {state_(0), state_(1), state_(2)},
            covariance_flat,
            lat,
            lon,
            alt,
            std::sqrt(state_.dot(state_)),
            std::max(0.0, 1.0 - (std::sqrt(state_.dot(state_)) / (1.0 + std::sqrt(state_.dot(state_)))) )
        };
    }

private:
    static double squarePositive(double value) {
        if (!std::isfinite(value) || value <= 0.0) {
            throw std::invalid_argument("process noise must be finite and positive");
        }
        const double variance = value * value;
        return variance;
    }

    static double degToRad(double degrees) {
        return degrees * 3.14159265358979323846 / 180.0;
    }

    Eigen::Vector3d state_;
    Eigen::Matrix3d covariance_;
    double human_height_m_;
    double human_height_stddev_m_;
    double process_noise_variance_;
    bool initialized_{false};
};

#endif
