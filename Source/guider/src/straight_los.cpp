#include "straight_los.h"
#include <ros/ros.h>
#include <utils/pointID.h>

StraightLOS::StraightLOS()
{

  ros::NodeHandle n;
  pub_PointID = n.advertise<pointID>("pointID", 1);
}

StraightLOS::~StraightLOS()
{
    if (ros::isStarted())
  {
    ros::shutdown();
    ros::waitForShutdown();
  }
}

void StraightLOS::setupLOS()
{
  numPoints = waypoints.size();
  // ROS_INFO_STREAM(numPoints);
  for (int i = 0 ; i < numPoints ; i++){
  //  ROS_INFO("\nway_x = %f\n way_y = %f ", waypoints[i].x, waypoints[i].y);     
  }
  if (numPoints > 1)
  {
    alpha_P.resize(numPoints - 1);
    s.resize(numPoints - 1);
    for (unsigned i = 0; i < numPoints - 1; i++)
    {
      double diff_x = waypoints[i + 1].x - waypoints[i].x;
      double diff_y = waypoints[i + 1].y - waypoints[i].y;
      alpha_P[i] = atan2(diff_y, diff_x);
      s[i] = diff_x * cos(alpha_P[i]) + diff_y * sin(alpha_P[i]);
    }
  }
}

void StraightLOS::resetLOS()
{
  waypoints.clear();
  pointId = 0;
}

bool StraightLOS::runLOS(const double& odomX, const double& odomY)
{
   utils::pointID _ID;
  _ID.pointID = pointId;
  pub_PointID.publish(_ID);
  double c_alpha = cos(alpha_P[pointId]);
  double s_alpha = sin(alpha_P[pointId]);
  double diff_x = odomX - waypoints[pointId].x;
  double diff_y = odomY - waypoints[pointId].y;
  // ROS_INFO("\nway_x = %f\n way_y = %f", waypoints[pointId].x, waypoints[pointId].y);
  // Calculate the along-track and cross-track errors
  alongTrackError = diff_x * c_alpha + diff_y * s_alpha;
  crossTrackError = -diff_x * s_alpha + diff_y * c_alpha;

  // Find desired heading
  double delta = (maxDelta - minDelta) * exp(-0.3* (crossTrackError * crossTrackError)) + minDelta;
  desiredHeading = alpha_P[pointId] + atan2(-crossTrackError, delta);
  desiredHeading = atan2(sin(desiredHeading), cos(desiredHeading));
 // ROS_INFO("desired Heading = %f", desiredHeading);
  // Head toward the next waypoint. For the nearly final point, discard point-switching scheme
  if (pointId < numPoints - 2)
  {
    if (fabs(s[pointId] - alongTrackError) < radius)
      pointId++;
  }
  else {
    if (fabs(s[pointId] - alongTrackError) < 1.0)
      pointId++;
  }

  // Terminate LOS if the final point has been reached
  return pointId < numPoints - 1;
}


