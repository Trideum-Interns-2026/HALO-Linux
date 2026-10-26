#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

namespace {

constexpr double max_offset_m = 2.0;
constexpr double arrival_tolerance_m = 0.25;
constexpr auto update_period = std::chrono::milliseconds(50);
constexpr auto movement_timeout = std::chrono::seconds(30);

volatile std::sig_atomic_t stop_requested = 0;

void handleSignal(int) {
    stop_requested = 1;
}

double parseOffset(const char* value, const char* name) {
    std::size_t parsed_characters = 0;
    const std::string input(value);
    const double offset = std::stod(input, &parsed_characters);
    if (parsed_characters != input.size() || !std::isfinite(offset) ||
        std::abs(offset) > max_offset_m) {
        throw std::invalid_argument(
            std::string(name) + " must be a number between -2 and 2 metres");
    }
    return offset;
}

bool validPosition(const mavsdk::Telemetry::PositionNed& position) {
    return std::isfinite(position.north_m) && std::isfinite(position.east_m) &&
        std::isfinite(position.down_m);
}

void requireSuccess(mavsdk::Offboard::Result result, const char* operation) {
    if (result != mavsdk::Offboard::Result::Success) {
        throw std::runtime_error(
            std::string(operation) + " failed (MAVSDK result " +
            std::to_string(static_cast<int>(result)) + ")");
    }
}

class OffboardGuard {
public:
    explicit OffboardGuard(mavsdk::Offboard& offboard) : offboard_(offboard) {}

    ~OffboardGuard() {
        if (active_) {
            const auto result = offboard_.stop();
            if (result != mavsdk::Offboard::Result::Success) {
                std::cerr << "Failed to stop Offboard during cleanup (MAVSDK result "
                          << static_cast<int>(result) << ")\n";
            }
        }
    }

    void markActive() {
        active_ = true;
    }

    mavsdk::Offboard::Result stop() {
        const auto result = offboard_.stop();
        if (result == mavsdk::Offboard::Result::Success) {
            active_ = false;
        }
        return result;
    }

private:
    mavsdk::Offboard& offboard_;
    bool active_{false};
};

}  // namespace

int main(int argc, char** argv) {
    try {
        if (argc != 3) {
            std::cerr << "Usage: move_to_waypoint <north_offset_m> <east_offset_m>\n"
                      << "Offsets are relative to the current position; each must "
                         "be within +/-2 m.\n";
            return 2;
        }

        const double north_offset_m = parseOffset(argv[1], "north offset");
        const double east_offset_m = parseOffset(argv[2], "east offset");
        if (north_offset_m == 0.0 && east_offset_m == 0.0) {
            throw std::invalid_argument("At least one horizontal offset must be non-zero");
        }

        std::signal(SIGINT, handleSignal);
        std::signal(SIGTERM, handleSignal);

        mavsdk::Mavsdk mavsdk{
            mavsdk::Mavsdk::Configuration{mavsdk::ComponentType::GroundStation}};
        mavsdk.set_timeout_s(3.0);
        const auto connection_result =
            mavsdk.add_any_connection("udpin://0.0.0.0:14540");
        if (connection_result != mavsdk::ConnectionResult::Success) {
            throw std::runtime_error(
                "MAVSDK connection failed (result " +
                std::to_string(static_cast<int>(connection_result)) + ")");
        }

        std::shared_ptr<mavsdk::System> system;
        for (int attempt = 0; attempt < 30 && !stop_requested && !system; ++attempt) {
            for (const auto& candidate : mavsdk.systems()) {
                if (candidate->is_connected()) {
                    system = candidate;
                    break;
                }
            }
            if (!system) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }
        if (!system) {
            throw std::runtime_error("No PX4 vehicle found on MAVSDK UDP 14540");
        }

        mavsdk::Telemetry telemetry{system};
        mavsdk::Offboard offboard{system};
        OffboardGuard offboard_guard{offboard};
        bool armed = false;
        bool in_air = false;
        for (int attempt = 0; attempt < 100 && !stop_requested; ++attempt) {
            armed = telemetry.armed();
            in_air = telemetry.in_air();
            if (armed && in_air) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if (stop_requested) {
            return 0;
        }
        if (!armed || !in_air) {
            throw std::runtime_error(
                "Vehicle state check failed after waiting 10 seconds "
                "(armed=" + std::string(armed ? "yes" : "no") +
                ", in_air=" + (in_air ? "yes" : "no") +
                "). Confirm QGroundControl shows the PX4 vehicle as armed and "
                "airborne. This script will not arm or take off.");
        }

        mavsdk::Telemetry::PositionVelocityNed current;
        bool have_position = false;
        for (int attempt = 0; attempt < 100 && !stop_requested; ++attempt) {
            current = telemetry.position_velocity_ned();
            if (validPosition(current.position)) {
                have_position = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if (!have_position) {
            throw std::runtime_error("Timed out waiting for a valid local position");
        }

        const auto heading = telemetry.heading().heading_deg;
        if (!std::isfinite(heading)) {
            throw std::runtime_error("No valid vehicle heading is available");
        }

        const mavsdk::Offboard::PositionNedYaw target{
            static_cast<float>(current.position.north_m + north_offset_m),
            static_cast<float>(current.position.east_m + east_offset_m),
            current.position.down_m,
            static_cast<float>(heading)};

        std::cout << "SITL waypoint: north " << north_offset_m << " m, east "
                  << east_offset_m << " m from current position. "
                  << "Press Ctrl+C to stop.\n";

        // PX4 requires setpoints to be streaming before Offboard can start.
        for (int attempt = 0; attempt < 20 && !stop_requested; ++attempt) {
            requireSuccess(offboard.set_position_ned(target), "Setting waypoint");
            std::this_thread::sleep_for(update_period);
        }
        if (stop_requested) {
            return 0;
        }
        requireSuccess(offboard.start(), "Starting Offboard mode");
        offboard_guard.markActive();

        const auto deadline = std::chrono::steady_clock::now() + movement_timeout;
        bool arrived = false;
        while (!stop_requested && std::chrono::steady_clock::now() < deadline) {
            if (!telemetry.armed() || !telemetry.in_air()) {
                throw std::runtime_error("Vehicle disarmed or left the air; stopping movement");
            }
            requireSuccess(offboard.set_position_ned(target), "Updating waypoint");

            current = telemetry.position_velocity_ned();
            if (validPosition(current.position)) {
                const double north_error =
                    target.north_m - current.position.north_m;
                const double east_error =
                    target.east_m - current.position.east_m;
                const double down_error =
                    target.down_m - current.position.down_m;
                if (std::hypot(north_error, east_error) <= arrival_tolerance_m &&
                    std::abs(down_error) <= arrival_tolerance_m) {
                    arrived = true;
                    break;
                }
            }
            std::this_thread::sleep_for(update_period);
        }

        const auto stop_result = offboard_guard.stop();
        if (stop_result != mavsdk::Offboard::Result::Success) {
            throw std::runtime_error(
                "Failed to stop Offboard mode (MAVSDK result " +
                std::to_string(static_cast<int>(stop_result)) + ")");
        }
        if (!arrived && !stop_requested) {
            throw std::runtime_error(
                "Waypoint was not reached within 30 seconds; PX4 returned to Hold mode");
        }

        std::cout << (arrived ? "Waypoint reached; PX4 returned to Hold mode.\n"
                              : "Stopped; PX4 returned to Hold mode.\n");
    } catch (const std::exception& error) {
        std::cerr << "Movement failed: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
