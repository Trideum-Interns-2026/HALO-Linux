cmake_minimum_required(VERSION 3.8)
project(geolocation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geolocation_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(geolocation_node src/geolocation_node.cpp)
target_include_directories(geolocation_node PRIVATE include)
ament_target_dependencies(geolocation_node
  geolocation_msgs
  rclcpp
  sensor_msgs
)

add_executable(building_risk_node src/building_risk_main.cpp)
target_include_directories(building_risk_node PRIVATE include)
ament_target_dependencies(building_risk_node
  ament_index_cpp
  geolocation_msgs
  rclcpp
)

# Offline filter tuner (no ROS): see tools/tune_position_filter.cpp.
add_executable(geolocation_tune tools/tune_position_filter.cpp)
target_include_directories(geolocation_tune PRIVATE include)

install(TARGETS geolocation_node building_risk_node geolocation_tune
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_position_estimator test/test_position_estimator.cpp)
  target_include_directories(test_position_estimator PRIVATE include)

  ament_add_gtest(test_building_occlusion test/test_building_occlusion.cpp)
  target_include_directories(test_building_occlusion PRIVATE include)

  ament_add_gtest(test_geolocation_node test/test_geolocation_node.cpp)
  target_include_directories(test_geolocation_node PRIVATE include)
  ament_target_dependencies(test_geolocation_node
    geolocation_msgs
    rclcpp
    sensor_msgs
  )

  ament_add_gtest(test_building_risk_node test/test_building_risk_node.cpp)
  target_include_directories(test_building_risk_node PRIVATE include)
  ament_target_dependencies(test_building_risk_node
    ament_index_cpp
    geolocation_msgs
    rclcpp
  )
endif()

ament_package()
