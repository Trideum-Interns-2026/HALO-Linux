// Offline tuner for Movement's gains. Runs the REAL controller (follow_logic.hpp) against a
// kinematic model of the drone and camera and searches for the smoothest settings that still keep
// the person centred. No ROS, no MAVSDK, no sim: `movement_tune` prints a table and the best
// parameters. The plant numbers below are assumptions until fitted to sim logs; retune after any
// change to the vehicle, camera rate or PX4 gains (see README, "Tuning").
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <optional>
#include <string>
#include <vector>

#include "movement/follow_logic.hpp"

using namespace movement;

namespace {

// ---- plant model (assumptions: fit them to sim logs, then retune) ----
constexpr double kStepS = 0.1;                 // the node's loop period
struct Plant {
    double velocity_lag_s{0.40};      // PX4 velocity loop: command -> body velocity
    double yaw_rate_lag_s{0.25};      // command -> yaw rate
    double detection_period_s{0.2};   // Perception output rate (camera ~5 Hz in the sim)
    double detection_latency_s{0.2};  // image -> detection
};
// The result must hold on all of these, not only the nominal one, so it survives the model
// being a bit wrong.
const Plant kPlants[] = {{0.40, 0.25, 0.2, 0.2}, {0.60, 0.40, 0.3, 0.3}, {0.25, 0.15, 0.1, 0.1}};
constexpr double kImageWidthPx = 1920.0;
constexpr double kFocalPx = 1395.0;            // x500 depth camera, 1080p

// ---- what counts as good ----
constexpr double kMaxBearingDeg = 12.0;        // person must stay this close to the image centre
constexpr double kSettleS = 6.0;               // ...after this long
constexpr double kMaxRadiusErrorM = 1.0;
constexpr double kMaxStepSettleS = 12.0;       // follow step: bearing < 5 deg and range within 0.5 m by then
constexpr double kTrackingWeight = 0.15;      // cost per deg^2 of mean squared bearing
constexpr double kJerkWeight = 0.02;          // cost of the squared rate of change of acceleration

CameraGeometry camera() {
    CameraGeometry c;
    c.center_x_px = kImageWidthPx / 2.0;
    c.width_px = kImageWidthPx;
    c.focal_x_px = kFocalPx;
    return c;
}

struct Drone {
    double x{0.0}, y{-9.0}, heading_deg{0.0};
    double fwd{0.0}, right{0.0}, yaw_rate{0.0};  // actual (lagged) body motion
};

struct Person {
    double x{0.0}, y{0.0}, vx{0.0}, vy{0.0};
};

double wrap180(double a) {
    while (a > 180.0) a -= 360.0;
    while (a < -180.0) a += 360.0;
    return a;
}

double bearingDeg(const Drone& d, const Person& p) {
    return wrap180(std::atan2(p.x - d.x, p.y - d.y) * 180.0 / kPi - d.heading_deg);
}

struct Sample {
    double x_px, range_m;
};

struct Metrics {
    double smoothness{0.0};      // integral of (rate of change of command)^2, normalised
    double jerk{0.0};            // integral of (rate of change of that)^2, normalised
    double worst_bearing{0.0};   // after the settle time
    double worst_radius_err{0.0};
    double unsettled_until{0.0};  // last time the person was off-centre (>5 deg) or off-range (>0.5 m)
    double bearing_sq_sum{0.0};  // for the mean squared bearing after settling
    int bearing_n{0};
    bool lost{false};            // person left the field of view
    double travelled_deg{0.0};
};

enum class Mode { FollowStep, FollowWalker, OrbitCcw, OrbitCw };

Metrics run(Mode mode, const FollowParams& p, const OrbitParams& o, double duration_s,
            const Plant& plant) {
    Drone d;
    Person person;
    if (mode == Mode::FollowStep) {
        d.x = 0.0; d.y = -9.0;  // starts 9 m away, 3 m too far
        d.heading_deg = 20.0;   // person starts 20 degrees off to the right
    } else if (mode == Mode::FollowWalker) {
        d.x = 0.0; d.y = -6.0;
        person.vx = 0.5;
    } else {
        d.x = 0.0; d.y = -6.0;
    }
    const bool orbit = mode == Mode::OrbitCcw || mode == Mode::OrbitCw;
    OrbitState ostate;
    CommandLimiter limiter;
    Metrics m;
    std::deque<std::pair<double, Sample>> pipeline;  // (arrival time, measurement)
    Sample held{kImageWidthPx / 2.0, 6.0};
    bool have_held = false;
    VelocityCommand prev;
    VelocityCommand prev_dc;
    double next_detection = 0.0, last_angle = std::atan2(-d.x, -d.y);
    const CameraGeometry cam = camera();

    for (double t = 0.0; t < duration_s; t += kStepS) {
        // Camera: a target fix every plant.detection_period_s, arriving plant.detection_latency_s
        // later (Perception, Tracking and the controller's input are all folded into these two).
        const double b = bearingDeg(d, person);
        if (t >= next_detection) {
            next_detection += plant.detection_period_s;
            if (std::abs(b) < 34.0) {  // inside the horizontal field of view
                pipeline.push_back({t + plant.detection_latency_s,
                                    {cam.center_x_px + kFocalPx * std::tan(b * kPi / 180.0),
                                     std::hypot(person.x - d.x, person.y - d.y)}});
            }
        }
        while (!pipeline.empty() && pipeline.front().first <= t) {
            held = pipeline.front().second;
            have_held = true;
            pipeline.pop_front();
        }
        if (std::abs(b) >= 34.0) {
            m.lost = true;
        }

        // Controller.
        VelocityCommand cmd;
        if (have_held) {
            if (orbit) {
                OrbitParams oo = o;
                oo.clockwise = mode == Mode::OrbitCw;
                cmd = computeOrbitCommand(held.x_px, held.range_m, cam, p, oo, ostate, kStepS);
            } else {
                cmd = computeFollowCommand(held.x_px, held.range_m, cam, p);
            }
        }
        cmd = limiter.apply(cmd, p, kStepS);

        // Smoothness: how fast the command changes (a proxy for jerk on the vehicle).
        const VelocityCommand dc{(cmd.forward_m_s - prev.forward_m_s) / kStepS,
                                 (cmd.right_m_s - prev.right_m_s) / kStepS,
                                 (cmd.yawspeed_deg_s - prev.yawspeed_deg_s) / kStepS};
        // Normalise each axis by its own scale so they weigh about the same.
        m.smoothness += kStepS * (std::pow(dc.forward_m_s / 1.0, 2) +
                                  std::pow(dc.right_m_s / 1.0, 2) +
                                  std::pow(dc.yawspeed_deg_s / 60.0, 2));
        m.jerk += kStepS * (std::pow((dc.forward_m_s - prev_dc.forward_m_s) / kStepS, 2) +
                            std::pow((dc.right_m_s - prev_dc.right_m_s) / kStepS, 2) +
                            std::pow((dc.yawspeed_deg_s - prev_dc.yawspeed_deg_s) / kStepS / 60.0,
                                     2));
        prev_dc = dc;
        prev = cmd;

        // Plant: first-order lags, then integrate.
        const double av = kStepS / (plant.velocity_lag_s + kStepS);
        const double ay = kStepS / (plant.yaw_rate_lag_s + kStepS);
        d.fwd += av * (cmd.forward_m_s - d.fwd);
        d.right += av * (cmd.right_m_s - d.right);
        d.yaw_rate += ay * (cmd.yawspeed_deg_s - d.yaw_rate);
        const double h = d.heading_deg * kPi / 180.0;
        d.x += (d.fwd * std::sin(h) + d.right * std::cos(h)) * kStepS;
        d.y += (d.fwd * std::cos(h) - d.right * std::sin(h)) * kStepS;
        d.heading_deg += d.yaw_rate * kStepS;
        person.x += person.vx * kStepS;
        person.y += person.vy * kStepS;

        {
            const double r0 = std::hypot(person.x - d.x, person.y - d.y);
            if (std::abs(bearingDeg(d, person)) > 5.0 || std::abs(r0 - p.safe_distance_m) > 0.5) {
                m.unsettled_until = t;
            }
        }
        if (t > kSettleS) {
            const double bd = bearingDeg(d, person);
            m.worst_bearing = std::max(m.worst_bearing, std::abs(bd));
            m.bearing_sq_sum += bd * bd;
            ++m.bearing_n;
            const double r = std::hypot(person.x - d.x, person.y - d.y);
            m.worst_radius_err = std::max(m.worst_radius_err, std::abs(r - p.safe_distance_m));
        }
        const double a = std::atan2(-d.x, -d.y);
        double delta = a - last_angle;
        while (delta > kPi) delta -= 2.0 * kPi;
        while (delta < -kPi) delta += 2.0 * kPi;
        m.travelled_deg += delta * 180.0 / kPi;
        last_angle = a;
    }
    return m;
}

struct Score {
    double value{0.0};
    bool ok{true};
    std::string why;
};

Score score(const FollowParams& p, const OrbitParams& o) {
    Score s;
    const struct { Mode mode; double duration; const char* name; } scenarios[] = {
        {Mode::FollowStep, 25.0, "follow step"}, {Mode::FollowWalker, 30.0, "follow walker"},
        {Mode::OrbitCcw, 40.0, "orbit ccw"}, {Mode::OrbitCw, 40.0, "orbit cw"}};
    double worst_plant = 0.0;
    for (const Plant& plant : kPlants) {
        double total = 0.0;
        for (const auto& sc : scenarios) {
            const Metrics m = run(sc.mode, p, o, sc.duration, plant);
            const double mean_sq = m.bearing_n > 0 ? m.bearing_sq_sum / m.bearing_n : 0.0;
            total += m.smoothness + kJerkWeight * m.jerk + kTrackingWeight * mean_sq;
            const bool orbit = sc.mode == Mode::OrbitCcw || sc.mode == Mode::OrbitCw;
            std::string why;
            if (m.lost) why = "person left the field of view";
            else if (m.worst_bearing > kMaxBearingDeg) why = "bearing too large";
            else if (m.worst_radius_err > kMaxRadiusErrorM) why = "radius not held";
            else if (sc.mode == Mode::FollowStep && m.unsettled_until > kMaxStepSettleS)
                why = "too slow to settle";
            else if (orbit && std::abs(m.travelled_deg) <
                                  0.8 * 360.0 * (sc.duration - kSettleS) / o.period_s)
                why = "orbit too slow";
            if (!why.empty()) {
                s.ok = false;
                s.why = std::string(sc.name) + ": " + why;
                s.value = 1e6;
                return s;
            }
        }
        worst_plant = std::max(worst_plant, total);
    }
    s.value = worst_plant;
    return s;
}

}  // namespace

int main(int argc, char** argv) {
    const OrbitParams orbit = sanitize(OrbitParams{true, false, kDefaultOrbitPeriodS,
                                                   kDefaultOrbitSpeedMps});
    // movement_tune yaw lateral forward ki a_fwd a_strafe a_yaw [smoothing_s]  -> score it
    if (argc == 8 || argc == 9) {
        FollowParams p;
        p.yaw_gain = std::atof(argv[1]); p.lateral_gain = std::atof(argv[2]);
        p.forward_gain = std::atof(argv[3]); p.orbit_yaw_ki = std::atof(argv[4]);
        p.max_forward_accel_m_s2 = std::atof(argv[5]); p.max_strafe_accel_m_s2 = std::atof(argv[6]);
        p.max_yaw_accel_deg_s2 = std::atof(argv[7]);
        if (argc == 9) p.command_smoothing_s = std::atof(argv[8]);
        const Score s = score(sanitize(p), orbit);
        std::printf("score %.2f  %s%s\n", s.value, s.ok ? "ok" : "FAILS: ", s.why.c_str());
        return s.ok ? 0 : 1;
    }
    const FollowParams baseline = sanitize(FollowParams{});
    const Score base = score(baseline, orbit);
    std::printf("baseline (current defaults): smoothness %.1f  %s%s\n", base.value,
                base.ok ? "ok" : "FAILS: ", base.why.c_str());

    struct Candidate { double value; FollowParams p; };
    std::vector<Candidate> good;
    const double yaw_gains[] = {0.3, 0.45, 0.6, 0.8, 1.0};
    const double lateral_gains[] = {0.2, 0.35, 0.5, 0.7};
    const double forward_gains[] = {0.2, 0.35, 0.5, 0.7, 1.0};
    const double kis[] = {5.0, 10.0, 20.0, 30.0, 45.0};
    const double fwd_acc[] = {0.15, 0.3, 0.5, 1.0, 0.0};
    const double strafe_acc[] = {0.15, 0.3, 0.6, 1.0, 0.0};
    const double yaw_acc[] = {15.0, 30.0, 60.0, 120.0, 0.0};
    const double smoothing[] = {0.0, 0.1, 0.2, 0.3};
    long tried = 0;
    for (double yg : yaw_gains) for (double lg : lateral_gains) for (double fg : forward_gains)
    for (double ki : kis) for (double fa : fwd_acc) for (double sa : strafe_acc)
    for (double ya : yaw_acc) for (double sm : smoothing) {
        FollowParams p;
        p.yaw_gain = yg; p.lateral_gain = lg; p.forward_gain = fg; p.orbit_yaw_ki = ki;
        p.max_forward_accel_m_s2 = fa; p.max_strafe_accel_m_s2 = sa; p.max_yaw_accel_deg_s2 = ya;
        p.command_smoothing_s = sm;
        p = sanitize(p);
        ++tried;
        const Score s = score(p, orbit);
        if (s.ok) good.push_back({s.value, p});
    }
    std::sort(good.begin(), good.end(),
              [](const Candidate& a, const Candidate& b) { return a.value < b.value; });
    std::printf("tried %ld combinations, %zu keep the person centred\n", tried, good.size());
    std::printf("%-9s %-5s %-5s %-5s %-4s %-5s %-5s %-5s %-5s\n", "score", "yaw", "lat", "fwd", "ki",
                "a_fwd", "a_str", "a_yaw", "tau");
    for (size_t i = 0; i < std::min<size_t>(good.size(), 8); ++i) {
        const auto& g = good[i];
        std::printf("%-9.1f %-5.2f %-5.2f %-5.2f %-4.0f %-5.1f %-5.1f %-5.0f %-5.2f\n", g.value,
                    g.p.yaw_gain, g.p.lateral_gain, g.p.forward_gain, g.p.orbit_yaw_ki,
                    g.p.max_forward_accel_m_s2, g.p.max_strafe_accel_m_s2,
                    g.p.max_yaw_accel_deg_s2, g.p.command_smoothing_s);
    }
    return good.empty() ? 1 : 0;
}
