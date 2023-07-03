#ifndef STRAIGHT_LOS_H
#define STRAIGHT_LOS_H

#include <base_los.h>
#include <ros/ros.h>
#include <utils/pointID.h>

using namespace utils;

class StraightLOS : public BaseLOS
{
public:
  StraightLOS();
  ~StraightLOS();

  std::vector<double> alpha_P;
  std::vector<double> s;
  unsigned pointId;
  unsigned long numPoints;
  ros::Publisher pub_PointID ;

  void setupLOS();
  void resetLOS();
  bool runLOS(const double& odomX, const double& odomY);

};

#endif // STRAIGHT_LOS_H
