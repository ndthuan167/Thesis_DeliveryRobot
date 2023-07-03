cmake_minimum_required(VERSION 2.8.3)
project(guider)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    rospy

    std_msgs
    utils
)

catkin_package()

include_directories(
    include
    ${catkin_INCLUDE_DIRS})

add_executable(guider
    src/guider.cpp
    src/base_los.cpp
    src/straight_los.cpp)
target_link_libraries(guider
    ${catkin_LIBRARIES})
