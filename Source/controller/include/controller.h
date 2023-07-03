#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <ros/ros.h>

#include <utils/Odometry.h>
#include <utils/Setpoint.h>
#include <utils/DiffVel.h>
#include <utils/Error.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>

#include <utils/ParamGet.h>
#include <utils/ParamSet.h>
#include <utils/mode_indoor.h>

#include "pid.h"

using namespace utils;
using namespace std;
using namespace sensor_msgs;
using namespace geometry_msgs;

#define THRESCHOLD  0.5 ;

class Controller
{
public:
  Controller();
  ~Controller();

  double controlling_period;
  double diff_rpm;
  double wheel_radius;
  float range_left;
  float range_right;
  float range_front[1000];
  float range_front_left[1000];
  float range_front_right[1000];
  float min_front;
  float min_front_left;
  float min_front_right ;
  uint8_t mode_in = 0.0 ;

  ros::Publisher pubDiffVel;
  ros::Subscriber subOdom;
  ros::Subscriber subSetpoint;
  ros::Timer loopControl;
  ros::Subscriber subRange;
  ros::Subscriber subPose;
  ros::Subscriber sub_mode ;

  ros::ServiceServer resSetHeadingPID;
  ros::ServiceServer resGetHeadingPID;

  PID speedPID;
  PID headingPID;

  double currHeading;
  double desiredSpeed;
  double desiredHeading;
  double currHeading_indoor;

  ros::Time lastSetpointTime;
  ros::Time lastControlUpdateTime;

  void onOdomCallBack(const Odometry::ConstPtr& msg);
  void onSetpointCallBack(const Setpoint::ConstPtr& msg);
  void onControlLoop(const ros::TimerEvent& event);
  void onLidarCallback(const LaserScan::ConstPtr& laser);
  void onPoseCallback(const PoseWithCovarianceStamped::ConstPtr& pos);
  void onModeCallback(const mode_indoor::ConstPtr& mode);

  bool onSetHeadingPIDCallBack(ParamSetRequest& req, ParamSetResponse& res);
  bool onGetHeadingPIDCallBack(ParamGetRequest& req, ParamGetResponse& res);

  inline bool compare_string(const char* str1, const char* str2) { return !strncmp(str1, str2, strlen(str2)); }
};

#endif // CONTROLLER_H
