#ifndef GUIDER_H
#define GUIDER_H

#include <ros/ros.h>

#include <utils/WaypointList.h>
#include <sensor_msgs/LaserScan.h>
#include <utils/Odometry.h>
#include <utils/Setpoint.h>
#include <utils/Error.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <utils/CommandLong.h>
#include <utils/ParamGet.h>
#include <utils/ParamSet.h>
#include <utils/SetMode.h>
#include <utils/mode_indoor.h>
#include <utils/Complete.h>
#include <utils/Position_avoid_obstacles.h>
#include <utils/pointID.h>
#include <utils/goal_indoor.h>
#include <utils/heading_plot.h>

#include <stdio.h>
#include <iostream>
#include <string>
#include <fstream>

#include "utils/geo.h"
#include "straight_los.h"
#include <tf/tf.h>

using namespace sensor_msgs;
using namespace std_msgs;
using namespace utils;
using namespace geometry_msgs;
using namespace std ;

class Guider
{
public:
  Guider();
  ~Guider();

  double ned_lat;
  double ned_lon;
  double guiding_period;
  uint8_t mode_in = 0.0 ;
  float AAA = 0 ;
  float X ,Y ;
  double currX ;
  double currY ;
  float currX_indoor = 0.0 ;
  float currY_indoor;
  float currX_outdoor;
  float currY_outdoor;
  float CurrX_indoor_plot = 0.0 ;
  bool exit_indoor ;

  float x_avoid, y_avoid ;

  bool complete = false ;
  bool obstacles = false ;
  bool indoor = false;

  uint8_t value = 0;
  uint8_t turn = 0;
  uint8_t pointId ;

  float lat_end_waypoint ;
  float lat_end_waypoint_set = 107746320.00 ;
  float complete_indoor ;
  float complete_indoor_set = 0.3 ;

  float waypoint_indoor_x[3];
  float waypoint_indoor_y[3];
  
   float waypoint_a_x[3];
  float waypoint_a_y[3];
  

  bool set_com_avoid = true;
  bool com_avoid ;

  float waypoint_x_0, waypoint_x_1, waypoint_x_2 ;
  float waypoint_y_0, waypoint_y_1, waypoint_y_2 ;


  double currHeading_indoor_x;
  double currHeading_indoor_y;
  double currHeading_indoor_z;
  double currHeading_indoor_w;

  bool complete_Avoid ;

  float goal_indoor_set ;

  ros::Publisher pubSetpoint;
  ros::Publisher pubError;
  ros::Publisher pubheadingplot;
  ros::Subscriber subOdom;
  ros::Subscriber subItemList;
  ros::Subscriber sub_mode ;
  ros::Publisher pub_complete ;
  ros::Subscriber sub_ID;

  ros::Timer loopGuidance;
  ros::Timer loopAddwaypointAvoid;
  ros::Timer loopAvoid;
  ros::Subscriber subPose;
  ros::Subscriber sub_avoid;
  ros::Subscriber sub_goal_indoor_set;

  ros::ServiceServer resStartMission;
  ros::ServiceServer resSetMode;
  ros::ServiceServer resSetSetpointParams;
  ros::ServiceServer resGetSetpointParams;
  ros::ServiceServer resSetLOSParams;
  ros::ServiceServer resGetLOSParams;

  StraightLOS straightLOSGuider;

  double desiredSpeed;
  double desiredHeading;
  double currHeading;

  bool isMissionStarted = false;
  std::string customMode;

  void onOdomCallBack(const Odometry::ConstPtr& msg);
  void onItemListCallBack(const WaypointList::ConstPtr& msg);
  void onGuidanceLoop(const ros::TimerEvent& event);
  void Waypoint_indoor();
  void onModeCallback(const mode_indoor::ConstPtr& mode);
  void onCallbackPose(const PoseWithCovarianceStamped::ConstPtr& pos) ;
  void addwaypoint(const ros::TimerEvent& event);
  void Avoid(const ros::TimerEvent& event);

  void onCallbackPosAvoid(const Position_avoid_obstacles::ConstPtr& pos_avoid);
  void onCallbackpointID(const pointID::ConstPtr& id);
  void onCallbackGoalindoor(const goal_indoor::ConstPtr& goal_in);

  bool onStartMissionCallBack(CommandLongRequest& req, CommandLongResponse& res);
  bool onSetModeCallBack(SetModeRequest& req, SetModeResponse& res);
  bool onSetSetpointParamsCallBack(ParamSetRequest& req, ParamSetResponse& res);
  bool onGetSetpointParamsCallBack(ParamGetRequest& req, ParamGetResponse& res);
  bool onSetLOSParamsCallBack(ParamSetRequest& req, ParamSetResponse& res);
  bool onGetLOSParamsCallBack(ParamGetRequest& req, ParamGetResponse& res);


  void publishSetpoint(const double& speed, const double& heading);
  void publishError(const double& xe, const double& ye);
  inline bool compare_string(const char* str1, const char* str2) { return !strncmp(str1, str2, strlen(str2)); }
};

#endif // GUIDER_H
