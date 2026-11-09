cmake_minimum_required(VERSION 3.16)
project(movement LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(perception_msgs REQUIRED)
find_package(MAVSDK REQUIRED)

# Node logic: no MAVSDK, so tests can link it with a fake vehicle.
add_library(movement_node_lib src/movement_node.cpp)
target_include_directories(movement_node_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(movement_node_lib rclcpp sensor_msgs perception_msgs)

add_executable(movement_node src/main.cpp src/mavsdk_vehicle.cpp)
target_link_libraries(movement_node movement_node_lib MAVSDK::mavsdk)

# Offline gain tuner: the real controller against a plant model (no ROS, MAVSDK or sim).
add_executable(movement_tune tools/tune_movement.cpp)
target_include_directories(movement_tune PRIVATE include)

install(TARGETS movement_node movement_tune DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_logic test/test_logic.cpp)
  target_include_directories(test_logic PRIVATE include)
  ament_add_gtest(test_node test/test_node.cpp)
  target_link_libraries(test_node movement_node_lib)
endif()

ament_package()
