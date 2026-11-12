// The one seam between the follow controller and the real vehicle link.
// The node only sees this interface, so tests replace it with a fake.
#pragma once

#include <optional>
#include <string>

namespace movement {

struct Status {
    bool ok{true};
    std::string what;  // reason when !ok
};

struct VehicleState {
    bool armed{false};
    bool in_air{false};
    // Heading in degrees from true north, clockwise, in [0, 360). nullopt = unknown.
    std::optional<double> heading_deg;
};

class VehicleInterface {
public:
    virtual ~VehicleInterface() = default;

    // Latest vehicle state, or nullopt while there is no live connection.
    [[nodiscard]] virtual std::optional<VehicleState> state() = 0;

    // Only used when the node is told to arm and take off itself.
    virtual Status arm() = 0;
    virtual Status takeoff() = 0;

    // Sends a zero setpoint, then switches PX4 to offboard (the original's order).
    virtual Status startOffboard() = 0;

    // Body-frame velocity: forward, right, down (m/s) and yaw rate (deg/s, + clockwise).
    virtual Status setVelocityBody(float forward_m_s, float right_m_s, float down_m_s,
                                   float yawspeed_deg_s) = 0;

    // Leaves offboard, which returns PX4 to Hold. Waits for the answer.
    virtual Status stopOffboard() = 0;
};

}  // namespace movement
