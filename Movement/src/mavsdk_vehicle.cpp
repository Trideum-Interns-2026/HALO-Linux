#include "movement/mavsdk_vehicle.hpp"

#include <chrono>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace movement {

namespace {

constexpr double kMavsdkTimeoutS = 3.0;  // as in the original
// Takeoff height (m above home) sent before every takeoff. 2.5 is PX4's default; the cap is the
// AGENTS.md limit (<= 10 m) and is enforced here, not left to the autopilot's parameter.
constexpr float kTakeoffAltitudeM = 2.5F;
constexpr float kMaxTakeoffAltitudeM = 10.0F;
static_assert(kTakeoffAltitudeM > 0.0F && kTakeoffAltitudeM <= kMaxTakeoffAltitudeM,
              "takeoff altitude must be within the 10 m limit");

// Attitude stream rate (Hz) requested once per connection for the heading.
constexpr double kAttitudeRateHz = 10.0;

template <typename R>
Status toStatus(R result, R success, const char* what) {
    if (result == success) {
        return {};
    }
    std::ostringstream os;
    os << what << " (MAVSDK result " << static_cast<int>(result) << ")";
    return {false, os.str()};
}

const Status kNoLink{false, "no vehicle connection"};

}  // namespace

MavsdkVehicle::MavsdkVehicle(const std::string& connection_url, double heading_timeout_s)
    : mavsdk_(mavsdk::Mavsdk::Configuration{mavsdk::ComponentType::GroundStation}),
      heading_freshness_(sanitizeHeadingTimeout(heading_timeout_s)) {
    mavsdk_.set_timeout_s(kMavsdkTimeoutS);
    const auto result = mavsdk_.add_any_connection(connection_url);
    if (result != mavsdk::ConnectionResult::Success) {
        throw std::runtime_error("MAVSDK connection to '" + connection_url +
                                 "' failed with code " +
                                 std::to_string(static_cast<int>(result)));
    }
}

bool MavsdkVehicle::connectIfNeeded() {
    if (system_ && system_->is_connected()) {
        return true;
    }
    system_.reset();
    action_.reset();
    offboard_.reset();
    telemetry_.reset();
    heading_freshness_.reset();  // a new link starts with no heading
    for (const auto& candidate : mavsdk_.systems()) {
        if (candidate->is_connected()) {
            system_ = candidate;
            action_ = std::make_unique<mavsdk::Action>(system_);
            offboard_ = std::make_unique<mavsdk::Offboard>(system_);
            telemetry_ = std::make_unique<mavsdk::Telemetry>(system_);
            // Non-blocking. A failure only means the stream keeps PX4's default rate, so it is
            // ignored; the heading is simply nullopt until attitude data arrives.
            telemetry_->set_rate_attitude_euler_async(kAttitudeRateHz,
                                                      [](mavsdk::Telemetry::Result) {});
            return true;
        }
    }
    return false;
}

std::optional<VehicleState> MavsdkVehicle::state() {
    if (!connectIfNeeded()) {
        return std::nullopt;
    }
    VehicleState s;
    s.armed = telemetry_->armed();
    s.in_air = telemetry_->in_air();
    const mavsdk::Telemetry::EulerAngle attitude = telemetry_->attitude_euler();
    const double now_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    // nullopt when the yaw is not finite or the attitude stream stopped advancing.
    const std::optional<double> yaw_deg =
        heading_freshness_.update(static_cast<double>(attitude.yaw_deg), attitude.timestamp_us,
                                  now_s);
    if (yaw_deg) {
        double heading = std::fmod(*yaw_deg, 360.0);
        if (heading < 0.0) {
            heading += 360.0;
        }
        if (heading >= 360.0) {  // fmod of a tiny negative value can round up to 360
            heading = 0.0;
        }
        s.heading_deg = heading;
    }
    return s;
}

Status MavsdkVehicle::arm() {
    if (!action_) {
        return kNoLink;
    }
    return toStatus(action_->arm(), mavsdk::Action::Result::Success, "arm failed");
}

Status MavsdkVehicle::takeoff() {
    if (!action_) {
        return kNoLink;
    }
    const Status altitude =
        toStatus(action_->set_takeoff_altitude(kTakeoffAltitudeM),
                 mavsdk::Action::Result::Success, "set takeoff altitude failed");
    if (!altitude.ok) {
        return altitude;  // never take off with an unbounded height
    }
    return toStatus(action_->takeoff(), mavsdk::Action::Result::Success, "takeoff failed");
}

Status MavsdkVehicle::startOffboard() {
    if (!offboard_) {
        return kNoLink;
    }
    const Status hold = toStatus(offboard_->set_velocity_body({0.0F, 0.0F, 0.0F, 0.0F}),
                                 mavsdk::Offboard::Result::Success,
                                 "prepare offboard setpoint failed");
    if (!hold.ok) {
        return hold;
    }
    return toStatus(offboard_->start(), mavsdk::Offboard::Result::Success,
                    "offboard start failed");
}

Status MavsdkVehicle::setVelocityBody(float forward_m_s, float right_m_s, float down_m_s,
                                      float yawspeed_deg_s) {
    if (!offboard_) {
        return kNoLink;
    }
    return toStatus(
        offboard_->set_velocity_body({forward_m_s, right_m_s, down_m_s, yawspeed_deg_s}),
        mavsdk::Offboard::Result::Success, "follow command failed");
}

Status MavsdkVehicle::stopOffboard() {
    if (!offboard_) {
        return kNoLink;
    }
    return toStatus(offboard_->stop(), mavsdk::Offboard::Result::Success,
                    "offboard stop failed");
}

}  // namespace movement
