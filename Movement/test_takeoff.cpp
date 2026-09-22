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

int main()
{
	Mavsdk mavsdk{Mavsdk::Configuration{ComponentType::GroundStation}};
	const auto connection_result = mavsdk.add_any_connection("udp://:14540");
	if (connection_result != ConnectionResult::Success) {
		std::cerr << "Connection failed: " << connection_result << '\n';
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
		std::cerr << "No PX4 vehicle found on UDP 14540\n";
		return EXIT_FAILURE;
	}

	Action action{system};
	Offboard offboard{system};
	Telemetry telemetry{system};

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
	std::this_thread::sleep_for(5s);

	offboard.set_velocity_body({0.0f, 0.0f, 0.0f, 45.0f});
	const auto offboard_result = offboard.start();
	if (offboard_result != Offboard::Result::Success) {
		std::cerr << "Offboard start failed: " << offboard_result << '\n';
		action.land();
		return EXIT_FAILURE;
	}

	std::this_thread::sleep_for(8s);
	offboard.set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});
	offboard.stop();

	action_result = action.land();
	if (action_result != Action::Result::Success) {
		std::cerr << "Land failed: " << action_result << '\n';
		return EXIT_FAILURE;
	}

	while (telemetry.in_air()) {
		std::this_thread::sleep_for(500ms);
	}

	std::cout << "Takeoff, 360-degree turn, and landing complete.\n";
	return EXIT_SUCCESS;
}