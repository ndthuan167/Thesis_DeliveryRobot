cmake_minimum_required(VERSION 2.8.3)
project(navigator)

add_compile_options(-std=c++11)
set(CMAKE_AUTOMOC ON)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    utils
)

find_package(Qt5 REQUIRED
  COMPONENTS
    Core
    SerialPort
)

catkin_package()

include_directories(
    include
    ${catkin_INCLUDE_DIRS})

add_executable(gps-ahrs_receiver
    src/gps-ahrs_receiver.cpp
    include/gps-ahrs_receiver.h)
target_link_libraries(gps-ahrs_receiver
    ${catkin_LIBRARIES}
    Qt5::Core
    Qt5::SerialPort)
