// VehicleInterface over MAVSDK / MAVLink. The only file pair that talks to the vehicle.
#pragma once

#include <memory>
#include <string>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include "movement/heading_freshness.hpp"
#include "movement/vehicle.hpp"

namespace movement {

class MavsdkVehicle : public VehicleInterface {
public:
    // Throws std::runtime_error if the connection URL is rejected. Finding the vehicle itself
    // is non-blocking: state() returns nullopt until it appears.
    // `heading_timeout_s`: the heading is dropped when the attitude timestamp stops advancing
    // for longer than this (sanitized to [kMinHeadingTimeoutS, kMaxHeadingTimeoutS]).
    explicit MavsdkVehicle(const std::string& connection_url,
                           double heading_timeout_s = kDefaultHeadingTimeoutS);

    std::optional<VehicleState> state() override;
    Status arm() override;
    Status takeoff() override;
    Status startOffboard() override;
    Status setVelocityBody(float forward_m_s, float right_m_s, float down_m_s,
                           float yawspeed_deg_s) override;
    Status stopOffboard() override;

private:
    bool connectIfNeeded();

    mavsdk::Mavsdk mavsdk_;
    HeadingFreshness heading_freshness_;
    std::shared_ptr<mavsdk::System> system_;
    std::unique_ptr<mavsdk::Action> action_;
    std::unique_ptr<mavsdk::Offboard> offboard_;
    std::unique_ptr<mavsdk::Telemetry> telemetry_;
};

}  // namespace movement
