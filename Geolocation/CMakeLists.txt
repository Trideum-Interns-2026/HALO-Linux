cmake_minimum_required(VERSION 3.8)
project(geolocation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
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

install(TARGETS geolocation_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_position_estimator test/test_position_estimator.cpp)
  target_include_directories(test_position_estimator PRIVATE include)

  ament_add_gtest(test_geolocation_node test/test_geolocation_node.cpp)
  target_include_directories(test_geolocation_node PRIVATE include)
  ament_target_dependencies(test_geolocation_node
    geolocation_msgs
    rclcpp
    sensor_msgs
  )
endif()

ament_package()
