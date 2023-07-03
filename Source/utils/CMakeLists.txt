cmake_minimum_required(VERSION 2.8.3)
project(utils)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    message_generation

    std_msgs
    geometry_msgs
)

add_message_files(
  FILES
    Odometry.msg
    Setpoint.msg
    Error.msg
    DiffVel.msg
    Encoder.msg
    KeyboardCommand.msg
    Waypoint.msg
    WaypointList.msg
    ParamValue.msg
    distance_msgs.msg
    mode_indoor.msg
    Complete.msg
    string_of_qrcode.msg
    Position_avoid_obstacles.msg
    pointID.msg
    goal_indoor.msg
    heading_plot.msg
    )

add_service_files(
  FILES
    CommandInt.srv
    CommandLong.srv
    ParamGet.srv
    ParamSet.srv
    SetMode.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
    INCLUDE_DIRS include)

include_directories(
    include
    ${catkin_INCLUDE_DIRS})

install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
