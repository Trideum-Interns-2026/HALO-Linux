cmake_minimum_required(VERSION 3.16)
project(halo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(MAVSDK REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(move ${CMAKE_SOURCE_DIR}/Src/move.cpp)
target_link_libraries(move MAVSDK::mavsdk)

include(CTest)
