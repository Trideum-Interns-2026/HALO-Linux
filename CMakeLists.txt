cmake_minimum_required(VERSION 3.16)
project(halo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(MAVSDK REQUIRED)

add_executable(main ${CMAKE_SOURCE_DIR}/Src/main.cpp)
target_include_directories(main PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/Include
    /usr/include/mavsdk
    /usr/local/include/mavsdk
)
target_link_libraries(main PRIVATE MAVSDK::mavsdk)