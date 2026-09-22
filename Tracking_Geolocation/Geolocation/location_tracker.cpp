#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <thread>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

using namespace mavsdk;
using namespace std::chrono_literals;

int main()
{
    Mavsdk mavsdk{Mavsdk::Configuration{ComponentType::GroundStation}};
    if (mavsdk.add_any_connection("udp://:14540") != ConnectionResult::Success) {
        std::cerr << "Connection failed.\n";
        return EXIT_FAILURE;
    }

    std::shared_ptr<System> system;
    for (int attempt = 0; attempt < 30 && !system; ++attempt) {
        for (const auto& candidate : mavsdk.systems()) {
            if (candidate->is_connected()) {
                system = candidate;
                break;
            }
        }
        std::this_thread::sleep_for(1s);
    }

    if (!system) {
        std::cerr << "No PX4 vehicle found on UDP 14540.\n";
        return EXIT_FAILURE;
    }

    Telemetry telemetry{system};
    telemetry.subscribe_position([](Telemetry::Position position) {
        std::cout << "Latitude: " << position.latitude_deg
                  << " deg, Longitude: " << position.longitude_deg
                  << " deg, Relative altitude: " << position.relative_altitude_m
                  << " m\n";
    });

    std::cout << "Tracking location. Press Ctrl+C to stop.\n";
    while (true) {
        std::this_thread::sleep_for(1s);
    }
}