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

add_executable(main ${CMAKE_SOURCE_DIR}/Src/main.cpp)
target_include_directories(main PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/Include
    /usr/include/mavsdk
    /usr/local/include/mavsdk
)
ament_target_dependencies(main
    geometry_msgs
    rclcpp
    sensor_msgs
    std_msgs
)
target_link_libraries(main Eigen3::Eigen MAVSDK::mavsdk)