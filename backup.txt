#include <chrono>
#include <iostream>
#include <thread>

#include "Groundstation/mavlink_interface.hpp"

using namespace std::chrono_literals;

int main()
{

    double TestLatitudeDeg = 68.3390;
    double TestLongitudeDeg = -94.0841;
    float TestRelativeAltitudeM = 129.0f;

    float TestVelocityXMps = 0.0f;
    float TestVelocityYMps = 0.0f;
    float TestVelocityZMps = 0.0f;

    int SendCount = 20;
    auto SendInterval = 500ms; 


    MavlinkSender sender("127.0.0.1", 14550);
    std::cout << "Sending " << SendCount << " fixed GLOBAL_POSITION_INT messages to "
              << "127.0.0.1:14550 (QGroundControl)...\n";

    for (int i = 0; i < SendCount; ++i) {
        sender.sendGlobalPosition(
            TestLatitudeDeg,
            TestLongitudeDeg,
            TestRelativeAltitudeM,
            TestVelocityXMps,
            TestVelocityYMps,
            TestVelocityZMps);

        std::cout << "[" << (i + 1) << "/" << SendCount << "] sent lat="
                  << TestLatitudeDeg << " lon=" << TestLongitudeDeg
                  << " alt=" << TestRelativeAltitudeM << "\n";

        std::this_thread::sleep_for(SendInterval);
    }

    std::cout << "Finished\n";
    return 0;
}