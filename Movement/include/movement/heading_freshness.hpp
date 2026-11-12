// Heading staleness guard: pure logic, no MAVSDK and no ROS.
// A telemetry stream can stop while the link stays up, which would freeze the last heading and
// make Movement steer toward a bearing that never closes. This class only passes a heading on
// while the telemetry timestamp keeps advancing.
#pragma once

#include <cmath>
#include <cstdint>
#include <optional>

namespace movement {

constexpr double kDefaultHeadingTimeoutS = 1.0;
constexpr double kMinHeadingTimeoutS = 0.1;
constexpr double kMaxHeadingTimeoutS = 10.0;

// A finite value within [kMinHeadingTimeoutS, kMaxHeadingTimeoutS]; non-finite gives the default.
inline double sanitizeHeadingTimeout(double timeout_s) {
    if (!std::isfinite(timeout_s)) {
        return kDefaultHeadingTimeoutS;
    }
    if (timeout_s < kMinHeadingTimeoutS) {
        return kMinHeadingTimeoutS;
    }
    if (timeout_s > kMaxHeadingTimeoutS) {
        return kMaxHeadingTimeoutS;
    }
    return timeout_s;
}

class HeadingFreshness {
public:
    explicit HeadingFreshness(double max_age_s) : max_age_s_(max_age_s) {}

    // Returns `heading_deg` while the sample's `timestamp_us` changed within `max_age_s` of
    // `now_s` (steady-clock seconds from the caller); exactly at the limit is still fresh.
    // nullopt for a non-finite heading (it is not recorded) or a stale stream. Any timestamp that
    // differs from the last one, also a smaller one, counts as a new sample.
    [[nodiscard]] std::optional<double> update(double heading_deg, uint64_t timestamp_us,
                                               double now_s) {
        if (!std::isfinite(heading_deg)) {
            return std::nullopt;
        }
        if (!have_sample_ || timestamp_us != last_timestamp_us_) {
            have_sample_ = true;
            last_timestamp_us_ = timestamp_us;
            last_change_s_ = now_s;
            return heading_deg;
        }
        if (now_s - last_change_s_ <= max_age_s_) {
            return heading_deg;
        }
        return std::nullopt;
    }

    // Forget everything (new connection): the next sample is a first sample.
    void reset() { have_sample_ = false; }

private:
    double max_age_s_;
    bool have_sample_{false};
    uint64_t last_timestamp_us_{0};
    double last_change_s_{0.0};
};

}  // namespace movement
