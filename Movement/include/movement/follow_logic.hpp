// Follow-person control math, ported unchanged from Lisa's Src/follow_person.cpp.
// No ROS and no MAVSDK in here, so it is unit-testable without either.
#pragma once

#include <algorithm>
#include <cmath>
#include <optional>

namespace movement {

constexpr double kPi = 3.14159265358979323846;

// Defaults, same values as the original controller.
constexpr double kDefaultDistanceM = 6.0;
constexpr double kMinDistanceM = 4.0;
constexpr double kMaxDistanceM = 9.0;
constexpr double kMaxForwardSpeedMps = 0.7;
constexpr double kMaxStrafeSpeedMps = 0.45;
constexpr double kMaxYawRateDegS = 45.0;
constexpr double kDefaultSearchYawRateDegS = 25.0;
constexpr double kDefaultSearchForwardMps = 0.35;
constexpr double kDefaultSearchStrafeMps = 0.45;
constexpr double kDefaultSearchOrbitHz = 0.25;
constexpr double kDefaultTargetPersonHeightM = 1.9;  // the Gazebo test person

// Controller gains, from the original.
constexpr double kForwardGain = 0.35;        // m/s per metre of distance error
constexpr double kLateralGain = 0.7;         // fraction of max strafe at the frame edge
constexpr double kSearchForwardFactor = 0.8;

// Hard caps. Parameters can tighten these but never loosen them.
constexpr double kForwardSpeedCeilingMps = 2.0;
constexpr double kStrafeSpeedCeilingMps = 1.5;
constexpr double kYawRateCeilingDegS = 90.0;
constexpr double kMinDistanceFloorM = 2.0;
constexpr double kMaxDistanceCeilingM = 30.0;
constexpr double kSearchOrbitCeilingHz = 1.0;

// Orbit mode (off by default). One lap takes `period_s`, so the sideways speed is
// 2 pi r / period_s: 1.26 m/s for 6 m in 30 s. That is above the follow strafe limit, so orbit has
// its own speed cap and hard ceiling; follow and the scan keep the limits above.
constexpr double kDefaultOrbitPeriodS = 30.0;
constexpr double kOrbitPeriodFloorS = 5.0;
constexpr double kOrbitPeriodCeilingS = 600.0;
constexpr double kDefaultOrbitSpeedMps = 1.5;
constexpr double kOrbitSpeedCeilingMps = 2.0;
// Orbit yaw is closed-loop on the pixel offset: proportional (the follow gain) plus an integral
// term that removes the steady offset a constant turn rate would otherwise leave.
constexpr double kOrbitYawIntegralGain = 30.0;  // deg/s per (normalised offset * second)

// Search around the last known direction (off by default: 0 s). On loss the vehicle holds its
// position and sweeps its nose over +-sweep_deg around where the person was last seen, then falls
// back to the original spiral scan.
constexpr double kDefaultLastKnownSearchS = 0.0;
constexpr double kDefaultLastKnownSweepDeg = 60.0;
constexpr double kDefaultLastKnownRateDegS = 20.0;
constexpr double kLastKnownSearchCeilingS = 60.0;
constexpr double kLastKnownSweepCeilingDeg = 180.0;

// Tunable gains (defaults are the values above) and their hard ceilings.
constexpr double kYawGain = 1.0;                 // fraction of max yaw rate at the frame edge
constexpr double kForwardGainCeiling = 2.0;
constexpr double kLateralGainCeiling = 1.0;
constexpr double kYawGainCeiling = 2.0;
constexpr double kOrbitYawIntegralGainCeiling = 120.0;
// Acceleration limits that smooth the commands. 0 = off (a command may jump), as originally.
constexpr double kDefaultAccelLimit = 0.0;
constexpr double kForwardAccelCeilingMps2 = 5.0;
constexpr double kStrafeAccelCeilingMps2 = 5.0;
constexpr double kYawAccelCeilingDegS2 = 360.0;

struct FollowParams {
    double safe_distance_m{kDefaultDistanceM};
    double min_distance_m{kMinDistanceM};
    double max_distance_m{kMaxDistanceM};
    double max_forward_m_s{kMaxForwardSpeedMps};
    double max_strafe_m_s{kMaxStrafeSpeedMps};
    double max_yaw_rate_deg_s{kMaxYawRateDegS};
    double search_yaw_rate_deg_s{kDefaultSearchYawRateDegS};
    double search_forward_m_s{kDefaultSearchForwardMps};
    double search_strafe_m_s{kDefaultSearchStrafeMps};
    double search_orbit_hz{kDefaultSearchOrbitHz};
    // Controller gains.
    double forward_gain{kForwardGain};                       // m/s per metre of range error
    double lateral_gain{kLateralGain};                       // fraction of max strafe at the edge
    double yaw_gain{kYawGain};                               // fraction of max yaw at the edge
    double orbit_yaw_ki{kOrbitYawIntegralGain};              // deg/s per normalised offset * s
    // Last-known-direction search; last_known_search_s = 0 keeps the original spiral scan only.
    double last_known_search_s{kDefaultLastKnownSearchS};
    double last_known_sweep_deg{kDefaultLastKnownSweepDeg};
    double last_known_rate_deg_s{kDefaultLastKnownRateDegS};
    // Smoothing: limits on how fast each command may change. 0 = no limit.
    double max_forward_accel_m_s2{kDefaultAccelLimit};
    double max_strafe_accel_m_s2{kDefaultAccelLimit};
    double max_yaw_accel_deg_s2{kDefaultAccelLimit};
};

struct OrbitParams {
    bool enabled{false};
    bool clockwise{false};  // seen from above; false = counter-clockwise
    double period_s{kDefaultOrbitPeriodS};
    double max_speed_m_s{kDefaultOrbitSpeedMps};
};

// Memory of the orbit's yaw controller. Reset it whenever the target is lost.
struct OrbitState {
    double yaw_integral{0.0};  // normalised offset * seconds, bounded
};

// Body-frame velocity command. Forward and right in m/s, yaw rate in deg/s (+ clockwise).
struct VelocityCommand {
    double forward_m_s{0.0};
    double right_m_s{0.0};
    double yawspeed_deg_s{0.0};
};

struct CameraGeometry {
    double center_x_px{0.0};  // principal point, K[2]
    double width_px{0.0};
    double height_px{0.0};    // image height; 0 = unknown, then no box is treated as clipped
    double focal_x_px{0.0};   // K[0]
    double focal_y_px{0.0};   // K[4]
};

[[nodiscard]] inline double clamp(double value, double min_value, double max_value) {
    return std::min(max_value, std::max(min_value, value));
}

[[nodiscard]] inline bool isFinite(double value) { return std::isfinite(value); }

// What the original camera_info callback accepted.
[[nodiscard]] inline bool isUsable(const CameraGeometry& c) {
    return isFinite(c.center_x_px) && isFinite(c.focal_x_px) && c.focal_x_px > 0.0 &&
           c.width_px > 0.0;
}

// Forces every limit into its hard bounds, whatever the parameters said. A non-finite value
// falls back to the original default.
[[nodiscard]] inline FollowParams sanitize(const FollowParams& in) {
    const auto fin = [](double v, double fallback) { return isFinite(v) ? v : fallback; };
    FollowParams p = in;
    p.min_distance_m = clamp(fin(p.min_distance_m, kMinDistanceM), kMinDistanceFloorM,
                             kMaxDistanceCeilingM);
    p.max_distance_m = clamp(fin(p.max_distance_m, kMaxDistanceM), p.min_distance_m,
                             kMaxDistanceCeilingM);
    p.safe_distance_m = clamp(fin(p.safe_distance_m, kDefaultDistanceM), p.min_distance_m,
                              p.max_distance_m);
    p.max_forward_m_s = clamp(fin(p.max_forward_m_s, kMaxForwardSpeedMps), 0.0,
                              kForwardSpeedCeilingMps);
    p.max_strafe_m_s = clamp(fin(p.max_strafe_m_s, kMaxStrafeSpeedMps), 0.0,
                             kStrafeSpeedCeilingMps);
    p.max_yaw_rate_deg_s = clamp(fin(p.max_yaw_rate_deg_s, kMaxYawRateDegS), 0.0,
                                 kYawRateCeilingDegS);
    p.search_yaw_rate_deg_s = clamp(fin(p.search_yaw_rate_deg_s, kDefaultSearchYawRateDegS),
                                    -p.max_yaw_rate_deg_s, p.max_yaw_rate_deg_s);
    p.search_forward_m_s = clamp(fin(p.search_forward_m_s, kDefaultSearchForwardMps),
                                 -p.max_forward_m_s, p.max_forward_m_s);
    p.search_strafe_m_s = clamp(fin(p.search_strafe_m_s, kDefaultSearchStrafeMps), 0.0,
                                p.max_strafe_m_s);
    p.search_orbit_hz = clamp(fin(p.search_orbit_hz, kDefaultSearchOrbitHz), 0.0,
                              kSearchOrbitCeilingHz);
    p.forward_gain = clamp(fin(p.forward_gain, kForwardGain), 0.0, kForwardGainCeiling);
    p.lateral_gain = clamp(fin(p.lateral_gain, kLateralGain), 0.0, kLateralGainCeiling);
    p.yaw_gain = clamp(fin(p.yaw_gain, kYawGain), 0.0, kYawGainCeiling);
    p.orbit_yaw_ki = clamp(fin(p.orbit_yaw_ki, kOrbitYawIntegralGain), 0.0,
                           kOrbitYawIntegralGainCeiling);
    p.last_known_search_s = clamp(fin(p.last_known_search_s, kDefaultLastKnownSearchS), 0.0,
                                  kLastKnownSearchCeilingS);
    p.last_known_sweep_deg = clamp(fin(p.last_known_sweep_deg, kDefaultLastKnownSweepDeg), 0.0,
                                   kLastKnownSweepCeilingDeg);
    p.last_known_rate_deg_s = clamp(fin(p.last_known_rate_deg_s, kDefaultLastKnownRateDegS), 0.0,
                                    p.max_yaw_rate_deg_s);
    p.max_forward_accel_m_s2 = clamp(fin(p.max_forward_accel_m_s2, kDefaultAccelLimit), 0.0,
                                     kForwardAccelCeilingMps2);
    p.max_strafe_accel_m_s2 = clamp(fin(p.max_strafe_accel_m_s2, kDefaultAccelLimit), 0.0,
                                    kStrafeAccelCeilingMps2);
    p.max_yaw_accel_deg_s2 = clamp(fin(p.max_yaw_accel_deg_s2, kDefaultAccelLimit), 0.0,
                                   kYawAccelCeilingDegS2);
    return p;
}

// Forces the orbit values into their hard bounds; non-finite falls back to the defaults.
[[nodiscard]] inline OrbitParams sanitize(const OrbitParams& in) {
    const auto fin = [](double v, double fallback) { return isFinite(v) ? v : fallback; };
    OrbitParams o = in;
    o.period_s = clamp(fin(o.period_s, kDefaultOrbitPeriodS), kOrbitPeriodFloorS,
                       kOrbitPeriodCeilingS);
    o.max_speed_m_s = clamp(fin(o.max_speed_m_s, kDefaultOrbitSpeedMps), 0.0,
                            kOrbitSpeedCeilingMps);
    return o;
}

// Sideways speed for one lap of `radius_m` in `period_s`, capped by max_speed_m_s.
// `o` must come from sanitize().
[[nodiscard]] inline double orbitSpeed(double radius_m, const OrbitParams& o) {
    return std::min(2.0 * kPi * radius_m / o.period_s, o.max_speed_m_s);
}

// Horizontal pixel offset of the target from the image centre, scaled to [-1, 1].
[[nodiscard]] inline double normalizedOffset(double target_x_px, const CameraGeometry& c) {
    return clamp((target_x_px - c.center_x_px) / (c.width_px * 0.5), -1.0, 1.0);
}

// Target in view: close the distance to `safe_distance_m`, strafe toward the target's side of
// the frame and yaw to keep it centred. `distance_m` is empty when no valid range exists, in
// which case there is no forward motion. `p` must come from sanitize().
[[nodiscard]] inline VelocityCommand computeFollowCommand(
    double target_x_px, std::optional<double> distance_m, const CameraGeometry& camera,
    const FollowParams& p) {
    VelocityCommand cmd;
    const double normalized = normalizedOffset(target_x_px, camera);

    if (distance_m) {
        const double d = clamp(*distance_m, p.min_distance_m, p.max_distance_m);
        const double distance_error = d - p.safe_distance_m;  // + = too far = move forward
        cmd.forward_m_s = clamp(distance_error * p.forward_gain, -p.max_forward_m_s,
                                p.max_forward_m_s);
    }

    const double lateral_error = clamp(normalized * p.lateral_gain, -1.0, 1.0);
    cmd.right_m_s = clamp(lateral_error * p.max_strafe_m_s, -p.max_strafe_m_s, p.max_strafe_m_s);
    cmd.yawspeed_deg_s = clamp(normalized * p.yaw_gain * p.max_yaw_rate_deg_s,
                               -p.max_yaw_rate_deg_s, p.max_yaw_rate_deg_s);
    return cmd;
}

// Target in view, orbit mode: circle the person at `safe_distance_m`. The sideways speed sets the
// lap time and the forward term holds the radius. The nose is never driven open-loop: yaw comes
// only from where the person is in the image (proportional + integral on the pixel offset), so the
// camera keeps the person centred wherever they are, even if they move or the circle is not
// perfect. (Facing a fixed point while circling it means the heading turns once per lap; that
// follows from the geometry and the controller finds the rate by itself.) Without a valid range
// the radius cannot be held, so it only keeps the person centred. `p` and `o` must come from
// sanitize(); `dt_s` is the time since the previous call.
[[nodiscard]] inline VelocityCommand computeOrbitCommand(
    double target_x_px, std::optional<double> distance_m, const CameraGeometry& camera,
    const FollowParams& p, const OrbitParams& o, OrbitState& state, double dt_s) {
    VelocityCommand cmd;
    const double normalized = normalizedOffset(target_x_px, camera);
    const double integral_limit =
        p.orbit_yaw_ki > 0.0 ? p.max_yaw_rate_deg_s / p.orbit_yaw_ki : 0.0;
    if (isFinite(dt_s) && dt_s > 0.0) {
        state.yaw_integral = clamp(state.yaw_integral + normalized * dt_s, -integral_limit,
                                   integral_limit);
    }
    const double yaw = normalized * p.yaw_gain * p.max_yaw_rate_deg_s +
                       p.orbit_yaw_ki * state.yaw_integral;
    cmd.yawspeed_deg_s = clamp(yaw, -p.max_yaw_rate_deg_s, p.max_yaw_rate_deg_s);
    if (!distance_m) {
        return cmd;
    }
    const double d = clamp(*distance_m, p.min_distance_m, p.max_distance_m);
    cmd.forward_m_s = clamp((d - p.safe_distance_m) * p.forward_gain, -p.max_forward_m_s,
                            p.max_forward_m_s);
    // Moving right circles the person counter-clockwise (seen from above); clockwise is left.
    const double direction = o.clockwise ? -1.0 : 1.0;
    cmd.right_m_s = clamp(direction * orbitSpeed(p.safe_distance_m, o), -kOrbitSpeedCeilingMps,
                          kOrbitSpeedCeilingMps);
    return cmd;
}

// Target lost, last-known search: hold position and turn the nose back and forth around the
// direction the person was last seen. `first_direction` is +1 (clockwise, to the right) or -1: the
// side the person was drifting toward when they vanished. The sweep goes that way to +sweep_deg,
// across to -sweep_deg, and back to the start, repeating; the rate is open loop but the pattern
// is symmetric, so the nose stays within +-sweep_deg of where it began. Only yaw is commanded.
// `p` must come from sanitize().
[[nodiscard]] inline VelocityCommand computeLastKnownSearchCommand(double elapsed_s,
                                                                   double first_direction,
                                                                   const FollowParams& p) {
    VelocityCommand cmd;
    if (!(p.last_known_rate_deg_s > 0.0) || !(p.last_known_sweep_deg > 0.0) ||
        !isFinite(elapsed_s) || elapsed_s < 0.0) {
        return cmd;
    }
    const double quarter_s = p.last_known_sweep_deg / p.last_known_rate_deg_s;
    const double phase = std::fmod(elapsed_s, 4.0 * quarter_s);
    const bool outward_or_home = phase < quarter_s || phase >= 3.0 * quarter_s;
    const double direction = first_direction < 0.0 ? -1.0 : 1.0;
    cmd.yawspeed_deg_s = (outward_or_home ? direction : -direction) * p.last_known_rate_deg_s;
    return cmd;
}

// True while the last-known search is still allowed to run.
[[nodiscard]] inline bool lastKnownSearchActive(double elapsed_s, const FollowParams& p) {
    return p.last_known_search_s > 0.0 && elapsed_s <= p.last_known_search_s;
}

// Target lost: spiral scan. Creep forward, sweep sideways on a sine and yaw at a constant rate.
// `elapsed_s` is the time since the scan began. `p` must come from sanitize().
[[nodiscard]] inline VelocityCommand computeSearchCommand(double elapsed_s,
                                                          const FollowParams& p) {
    VelocityCommand cmd;
    const double orbit_phase = 2.0 * kPi * p.search_orbit_hz * elapsed_s;
    cmd.forward_m_s = p.search_forward_m_s * kSearchForwardFactor;
    cmd.right_m_s = std::sin(orbit_phase) * p.search_strafe_m_s;
    cmd.yawspeed_deg_s = p.search_yaw_rate_deg_s;
    return cmd;
}

// Smooths a command stream: each component may change by at most limit * dt per step, so a
// target that jumps across the frame does not jerk the vehicle. A limit of 0 leaves that
// component untouched. Starts from rest; call reset() when offboard (re)starts.
class CommandLimiter {
public:
    void reset() { last_ = VelocityCommand{}; }
    [[nodiscard]] VelocityCommand apply(const VelocityCommand& wanted, const FollowParams& p,
                                        double dt_s) {
        if (!isFinite(dt_s) || dt_s <= 0.0) {
            return last_ = wanted;
        }
        last_.forward_m_s = step(last_.forward_m_s, wanted.forward_m_s,
                                 p.max_forward_accel_m_s2 * dt_s);
        last_.right_m_s = step(last_.right_m_s, wanted.right_m_s, p.max_strafe_accel_m_s2 * dt_s);
        last_.yawspeed_deg_s = step(last_.yawspeed_deg_s, wanted.yawspeed_deg_s,
                                    p.max_yaw_accel_deg_s2 * dt_s);
        return last_;
    }

private:
    // max_change <= 0 means unlimited.
    static double step(double from, double to, double max_change) {
        return max_change > 0.0 ? from + clamp(to - from, -max_change, max_change) : to;
    }
    VelocityCommand last_;
};

// Perception publishes a pixel box, not a range, so Movement derives the two numbers the
// controller needs, the same way the original detector did.
[[nodiscard]] inline double boxCenterX(double x_min_px, double x_max_px) {
    return 0.5 * (x_min_px + x_max_px);
}

// A box touching the top or bottom of the image is cut off (a person whose feet are below the
// frame, say), so its height is too small and the pinhole range far too big. Such a box says
// nothing reliable about distance.
constexpr double kFrameEdgeMarginPx = 2.0;
[[nodiscard]] inline bool boxTouchesFrameEdge(double y_min_px, double y_max_px,
                                              const CameraGeometry& c) {
    return c.height_px > 0.0 &&
           (y_min_px <= kFrameEdgeMarginPx || y_max_px >= c.height_px - kFrameEdgeMarginPx);
}

// Pinhole range: person_height_m * fy / box_height_px. Empty when the box or intrinsics are
// unusable (the original accepted only finite, positive ranges).
[[nodiscard]] inline std::optional<double> rangeFromBox(
    double y_min_px, double y_max_px, double focal_y_px, double person_height_m) {
    const double box_height = y_max_px - y_min_px;
    if (!isFinite(box_height) || !isFinite(focal_y_px) || !isFinite(person_height_m) ||
        box_height <= 0.0 || focal_y_px <= 0.0 || person_height_m <= 0.0) {
        return std::nullopt;
    }
    const double range = person_height_m * focal_y_px / box_height;
    if (!isFinite(range) || range <= 0.0) {
        return std::nullopt;
    }
    return range;
}

}  // namespace movement
