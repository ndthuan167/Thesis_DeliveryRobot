cmake_minimum_required(VERSION 2.8.3)
project(simulation)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    rospy
    message_filters
    gazebo_ros
    gazebo_plugins

    sensor_msgs
    nav_msgs
    tf
    utils
)

if(POLICY CMP0054)
  cmake_policy(SET CMP0054 OLD)
endif()


find_package(gazebo REQUIRED)

catkin_package()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS})

add_library(diff_motors_plugin
    src/diff_motors_plugin.cc)
target_link_libraries(diff_motors_plugin
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES})

add_library(ahrs_filter
    src/ahrs_filter.cc)
target_link_libraries(ahrs_filter
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES})

add_library(gps_plugin
    src/gps_plugin.cc)
target_link_libraries(gps_plugin
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES})

add_library(imu_plugin
    src/imu_plugin.cc)
target_link_libraries(imu_plugin
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES})

add_library(magnet_plugin
    src/magnet_plugin.cc)
target_link_libraries(magnet_plugin
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES})

add_library(p3d_plugin
    src/p3d_plugin.cc)
target_link_libraries(p3d_plugin
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES})

