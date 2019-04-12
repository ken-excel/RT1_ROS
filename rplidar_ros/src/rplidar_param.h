#ifndef _RPLIDAR_PARAM_H_
#define _RPLIDAR_PARAM_H_

#include <math.h>

#define DEG2RAD(x) ((x)*M_PI/180)

class RT1Param
{
public:
  double height;
  double incline; 
  double distance; 
  double range;

  RT1Param()
  {
    height = 0.4905;
    incline = 20; 
    distance = height/tan(DEG2RAD(incline));
    range = height/sin(DEG2RAD(incline));
  }

  bool load(const ros::NodeHandle &nh)
  {
    nh.param<double>("height", height, height);
    nh.param<double>("incline", incline, incline);
    nh.param<double>("distance", distance, distance);
    nh.param<double>("range", range, range);
    return true;
  }
};

#endif // _RPLIDAR_PARAM_H_