#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <thread>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

using namespace mavsdk;
using namespace std::chrono_literals;

namespace {
constexpr float TwoFeetMeters = 0.6096f;
constexpr float SpeedMetersPerSecond = 0.3f;
constexpr auto MovementDuration = std::chrono::milliseconds{
    static_cast<int>(TwoFeetMeters / SpeedMetersPerSecond * 1000.0f)};

std::shared_ptr<System> connect_to_vehicle(Mavsdk& mavsdk)
{
    if (mavsdk.add_any_connection("udp://:14540") != ConnectionResult::Success) {
        return nullptr;
    }

    for (int attempt = 0; attempt < 30; ++attempt) {
        for (const auto& candidate : mavsdk.systems()) {
            if (candidate->is_connected()) {
                return candidate;
            }
        }
        std::this_thread::sleep_for(1s);
    }
    return nullptr;
}
} // namespace

int main()
{
    Mavsdk mavsdk{Mavsdk::Configuration{ComponentType::GroundStation}};
    const auto system = connect_to_vehicle(mavsdk);
    if (!system) {
        std::cerr << "No PX4 vehicle found on UDP 14540.\n";
        return EXIT_FAILURE;
    }

    Action action{system};
    Offboard offboard{system};
    Telemetry telemetry{system};

    telemetry.subscribe_position([](Telemetry::Position position) {
        std::cout << "Latitude: " << position.latitude_deg
                  << " deg, Longitude: " << position.longitude_deg
                  << " deg, Relative altitude: " << position.relative_altitude_m
                  << " m\n";
    });

    while (!telemetry.health_all_ok()) {
        std::cout << "Waiting for PX4 health checks...\n";
        std::this_thread::sleep_for(1s);
    }

    auto action_result = action.arm();
    if (action_result != Action::Result::Success) {
        std::cerr << "Arm failed: " << action_result << '\n';
        return EXIT_FAILURE;
    }

    action_result = action.takeoff();
    if (action_result != Action::Result::Success) {
        std::cerr << "Takeoff failed: " << action_result << '\n';
        return EXIT_FAILURE;
    }

    while (!telemetry.in_air()) {
        std::this_thread::sleep_for(200ms);
    }
    std::this_thread::sleep_for(3s);

    offboard.set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});
    if (offboard.start() != Offboard::Result::Success) {
        std::cerr << "Offboard start failed.\n";
        action.land();
        return EXIT_FAILURE;
    }

    std::cout << "Moving forward approximately two feet.\n";
    offboard.set_velocity_body({SpeedMetersPerSecond, 0.0f, 0.0f, 0.0f});
    std::this_thread::sleep_for(MovementDuration);

    std::cout << "Moving backward approximately two feet.\n";
    offboard.set_velocity_body({-SpeedMetersPerSecond, 0.0f, 0.0f, 0.0f});
    std::this_thread::sleep_for(MovementDuration);

    offboard.set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});
    offboard.stop();

    std::cout << "Landing.\n";
    action_result = action.land();
    if (action_result != Action::Result::Success) {
        std::cerr << "Land failed: " << action_result << '\n';
        return EXIT_FAILURE;
    }

    while (telemetry.in_air()) {
        std::this_thread::sleep_for(500ms);
    }

    std::cout << "Flight and telemetry test complete.\n";
    return EXIT_SUCCESS;
}