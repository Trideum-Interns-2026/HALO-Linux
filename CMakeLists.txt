cmake_minimum_required(VERSION 3.16)
project(halo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(MAVSDK REQUIRED)

file(GLOB_RECURSE HALO_SOURCES CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/*.cpp")

foreach(source IN LISTS HALO_SOURCES)
    get_filename_component(target "${source}" NAME_WE)
    add_executable(${target} "${source}")
    target_link_libraries(${target} PRIVATE MAVSDK::mavsdk)
endforeach()