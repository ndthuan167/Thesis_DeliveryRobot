cmake_minimum_required(VERSION 2.8.3)
project(gcs_transceiver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    mavconn
    utils
)

catkin_package()

include_directories(
    include
    ${catkin_INCLUDE_DIRS})

add_executable(gcs_transceiver
    src/gcs_transceiver.cpp
    src/mission_protocol.cpp
    src/command_protocol.cpp
    src/parameter_protocol.cpp)
target_link_libraries(gcs_transceiver
    ${catkin_LIBRARIES})
