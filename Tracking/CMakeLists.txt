cmake_minimum_required(VERSION 3.8)
project(tracking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geolocation_msgs REQUIRED)
find_package(perception_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tracking_msgs REQUIRED)

add_executable(tracking_node src/tracking_node.cpp)
target_include_directories(tracking_node PRIVATE include)
ament_target_dependencies(tracking_node
  geolocation_msgs
  perception_msgs
  px4_msgs
  rclcpp
  sensor_msgs
  tracking_msgs
)

install(TARGETS tracking_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_target_estimator test/test_target_estimator.cpp)
  target_include_directories(test_target_estimator PRIVATE include)

  ament_add_gtest(test_tracking_node test/test_tracking_node.cpp)
  target_include_directories(test_tracking_node PRIVATE include)
  ament_target_dependencies(test_tracking_node
    geolocation_msgs
    perception_msgs
    px4_msgs
    rclcpp
    sensor_msgs
    tracking_msgs
  )
endif()

ament_package()
