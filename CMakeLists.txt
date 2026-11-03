cmake_minimum_required(VERSION 3.16)
project(halo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(geolocation ${CMAKE_SOURCE_DIR}/Geolocation/src/geolocation_node.cpp)
target_include_directories(geolocation PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/Geolocation/include
)
ament_target_dependencies(geolocation
    geometry_msgs
    rclcpp
    sensor_msgs
    std_msgs
)
target_link_libraries(geolocation Eigen3::Eigen)

add_executable(tracking ${CMAKE_SOURCE_DIR}/Tracking/src/tracking_node.cpp)
target_include_directories(tracking PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/Tracking/include
)
ament_target_dependencies(tracking
    geometry_msgs
    rclcpp
    sensor_msgs
    std_msgs
)
target_link_libraries(tracking Eigen3::Eigen)

include(CTest)
if(BUILD_TESTING)
    add_executable(drone_gps_kalman_filter_test
        ${CMAKE_SOURCE_DIR}/Geolocation/tests/drone_gps_kalman_filter_test.cpp)
    target_include_directories(drone_gps_kalman_filter_test PRIVATE
        ${CMAKE_SOURCE_DIR}/Geolocation/include
    )
    target_link_libraries(drone_gps_kalman_filter_test Eigen3::Eigen)
    add_test(NAME drone_gps_kalman_filter_test COMMAND drone_gps_kalman_filter_test)
endif()