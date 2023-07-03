#ifndef BASE_LOS_H
#define BASE_LOS_H

#include <cmath>
#include <vector>

class BaseLOS
{
public:
  BaseLOS();
  virtual ~BaseLOS();

  struct Point
  {
    inline Point() : x(0), y(0) {}
    inline Point(double _x, double _y) : x(_x), y(_y) {}

    double x;
    double y;
  };
  static std::vector<Point> waypoints;

  static double radius;
  static double minDelta ;
  static double maxDelta;
  static double beta;

  double crossTrackError;
  double alongTrackError;
  double desiredHeading;

  virtual void setupLOS();
  virtual void resetLOS();
  virtual bool runLOS(const double& odomX, const double& odomY);
};

#endif // BASE_LOS_H
