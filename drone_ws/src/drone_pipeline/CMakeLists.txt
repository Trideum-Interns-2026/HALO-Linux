cmake_minimum_required(VERSION 3.8)
project(drone_pipeline)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(drone_msgs REQUIRED)
find_package(Threads REQUIRED)

include_directories(include)

foreach(node
    world_sim camera_sim px4_sim
    perception_node geolocation_node poi_tracker_node
    movement_node web_node)
  add_executable(${node} src/${node}.cpp)
  ament_target_dependencies(${node} rclcpp sensor_msgs drone_msgs)
  target_link_libraries(${node} Threads::Threads)
  install(TARGETS ${node} DESTINATION lib/${PROJECT_NAME})
endforeach()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_package()
