#include "movement/mavsdk_vehicle.hpp"

#include <sstream>
#include <stdexcept>

namespace movement {

namespace {

constexpr double kMavsdkTimeoutS = 3.0;  // as in the original

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

MavsdkVehicle::MavsdkVehicle(const std::string& connection_url)
    : mavsdk_(mavsdk::Mavsdk::Configuration{mavsdk::ComponentType::GroundStation}) {
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
    for (const auto& candidate : mavsdk_.systems()) {
        if (candidate->is_connected()) {
            system_ = candidate;
            action_ = std::make_unique<mavsdk::Action>(system_);
            offboard_ = std::make_unique<mavsdk::Offboard>(system_);
            telemetry_ = std::make_unique<mavsdk::Telemetry>(system_);
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
