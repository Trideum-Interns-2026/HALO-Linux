// Offline tuner for Geolocation's position filter. Feeds the REAL PositionEstimator a noisy GPS
// stream from a known trajectory and scores error against the truth. The simulator's GPS has no
// noise at all (the x500 navsat sensor has no <noise>), so tuning against it would only turn the
// filter into a pass-through; this uses the noise of a real receiver instead (assumed, see below).
// Retune when the receiver, its rate, or the vehicle's dynamics change (see README "Tuning").
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

#include "geolocation/position_estimator.hpp"

using namespace geolocation;

namespace {

// ---- assumptions: a consumer GPS receiver ----
constexpr double kGpsHorizontalStddevM = 1.0;   // per axis; also the node's default fallback
constexpr double kGpsVerticalStddevM = 1.5;
constexpr double kOriginLatDeg = 47.397971, kOriginLonDeg = 8.546164, kOriginAltM = 0.0;
constexpr double kRadiusM = 6.0;                // orbit radius (the follow distance)

struct Scenario {
    const char* name;
    double lap_s;       // 0 = hover
    double rate_hz;     // GPS fix rate
    double duration_s;
};

GeodeticPosition fromEnu(double east, double north, double up) {
    const double lat = kOriginLatDeg + north / 111320.0;
    const double lon = kOriginLonDeg +
                       east / (111320.0 * std::cos(kOriginLatDeg * kPi / 180.0));
    return {lat, lon, kOriginAltM + up};
}

struct Result {
    double rms_error_m{0.0};      // horizontal, filtered
    double raw_rms_error_m{0.0};  // horizontal, the GPS fix itself
    double rms_lag_m{0.0};        // error along the direction of travel
    double coverage{0.0};         // fraction of fixes whose true error is within the reported 3 sigma
};

Result run(const Scenario& sc, double process_noise, double fallback_h, unsigned seed) {
    PositionEstimator filter(fallback_h, kGpsVerticalStddevM, process_noise);
    std::mt19937 rng(seed);
    std::normal_distribution<double> noise(0.0, 1.0);
    const double dt = 1.0 / sc.rate_hz;
    const double omega = sc.lap_s > 0.0 ? 2.0 * kPi / sc.lap_s : 0.0;
    double sum_sq = 0.0, raw_sq = 0.0, lag_sq = 0.0;
    int n = 0, covered = 0;
    const GeodeticPosition origin = fromEnu(0.0, 0.0, 0.0);
    // Local ENU of the first fix is the filter's origin, so compare in ENU relative to it.
    double first_e = 0.0, first_n = 0.0;
    bool have_first = false;
    for (double t = 0.0; t < sc.duration_s; t += dt) {
        const double e = kRadiusM * std::cos(omega * t), nn = kRadiusM * std::sin(omega * t);
        const double me = e + kGpsHorizontalStddevM * noise(rng);
        const double mn = nn + kGpsHorizontalStddevM * noise(rng);
        const double mu = kGpsVerticalStddevM * noise(rng);
        const Estimate est = filter.update(fromEnu(me, mn, mu), {0.0, 0.0, 0.0}, dt);
        (void)origin;
        if (!have_first) { first_e = me; first_n = mn; have_first = true; }
        if (t < 10.0) continue;  // let the filter settle
        // The estimate comes back as geodetic; convert to metres around the origin.
        const double est_e = (est.position.longitude_deg - kOriginLonDeg) *
                             111320.0 * std::cos(kOriginLatDeg * kPi / 180.0);
        const double est_n = (est.position.latitude_deg - kOriginLatDeg) * 111320.0;
        const double err_e = est_e - e, err_n = est_n - nn;
        const double err = std::hypot(err_e, err_n);
        sum_sq += err * err;
        raw_sq += (me - e) * (me - e) + (mn - nn) * (mn - nn);
        if (omega > 0.0) {  // unit tangent of the motion
            const double te = -std::sin(omega * t), tn = std::cos(omega * t);
            const double along = err_e * te + err_n * tn;
            lag_sq += along * along;
        }
        const double three_sigma = est.error_ecef_3sigma_m[0];  // an upper bound of horizontal
        if (err <= std::hypot(est.error_ecef_3sigma_m[0], est.error_ecef_3sigma_m[1],
                              est.error_ecef_3sigma_m[2])) ++covered;
        (void)three_sigma;
        ++n;
    }
    (void)first_e; (void)first_n;
    Result r;
    r.rms_error_m = std::sqrt(sum_sq / n);
    r.raw_rms_error_m = std::sqrt(raw_sq / n);
    r.rms_lag_m = std::sqrt(lag_sq / n);
    r.coverage = static_cast<double>(covered) / n;
    return r;
}

}  // namespace

int main() {
    const Scenario scenarios[] = {{"hover, 5 Hz GPS", 0.0, 5.0, 120.0},
                                  {"orbit 30 s/lap, 5 Hz GPS", 30.0, 5.0, 120.0},
                                  {"orbit 30 s/lap, 30 Hz GPS (sim rate)", 30.0, 30.0, 120.0}};
    const double q_grid[] = {0.01, 0.03, 0.1, 0.25, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0};
    std::printf("process noise q (m^2/s) vs horizontal RMS error (m), GPS sigma %.1f m, "
                "averaged over 8 seeds\n", kGpsHorizontalStddevM);
    std::printf("%-8s", "q");
    for (const auto& sc : scenarios) std::printf(" | %-36s", sc.name);
    std::printf("\n");
    double best_q = 0.0, best_cost = 1e9;
    for (double q : q_grid) {
        std::printf("%-8.2f", q);
        double cost = 0.0;
        bool covered = true;
        for (const auto& sc : scenarios) {
            double rms = 0.0, raw = 0.0, lag = 0.0, cov = 0.0;
            for (unsigned seed = 1; seed <= 8; ++seed) {
                const Result r = run(sc, q, kGpsHorizontalStddevM, seed);
                rms += r.rms_error_m / 8; raw += r.raw_rms_error_m / 8;
                lag += r.rms_lag_m / 8; cov += r.coverage / 8;
            }
            std::printf(" | err %.2f (raw %.2f) lag %.2f cov %.2f", rms, raw, lag, cov);
            cost += rms;
            covered = covered && cov >= 0.95;
        }
        std::printf("%s\n", covered ? "" : "  (error bars too small)");
        if (covered && cost < best_cost) { best_cost = cost; best_q = q; }
    }
    std::printf("\nbest q with honest error bars (coverage >= 95%%): %.2f m^2/s "
                "(summed RMS %.2f m)\n", best_q, best_cost);
    return best_q > 0.0 ? 0 : 1;
}
