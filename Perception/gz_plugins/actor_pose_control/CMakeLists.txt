# Builds libActorPoseControl.so, the Gazebo system walk_person.py uses to
# drive the animated person. walk_person.py builds it automatically when
# it's missing or out of date; by hand:
#   cmake -S Perception/gz_plugins/actor_pose_control -B Perception/gz_plugins/actor_pose_control/build
#   cmake --build Perception/gz_plugins/actor_pose_control/build
cmake_minimum_required(VERSION 3.16)
project(actor_pose_control CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

add_library(ActorPoseControl SHARED ActorPoseControl.cc)
target_link_libraries(ActorPoseControl PRIVATE
  gz-sim8::gz-sim8
  gz-plugin2::register
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10)
