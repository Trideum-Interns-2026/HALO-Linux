cmake_minimum_required(VERSION 3.16)
project(test_takeoff LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(MAVSDK REQUIRED)

add_executable(test_takeoff test_takeoff.cpp)

target_link_libraries(
    test_takeoff
    PRIVATE
    MAVSDK::mavsdk
)

