cmake_minimum_required(VERSION 3.16)
project(geolocation_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(MAVSDK REQUIRED)

add_executable(location_tracker location_tracker.cpp)
target_link_libraries(location_tracker PRIVATE MAVSDK::mavsdk)

add_executable(takeoff_forward_back takeoff_forward_back.cpp)
target_link_libraries(takeoff_forward_back PRIVATE MAVSDK::mavsdk)
