#ifndef _RT1_NAV_PARAM_H_
#define _RT1_NAV_PARAM_H_

#include <math.h>

class RT1NavParam
{
public:
  double origin_x, origin_y;
  double bound_x_min, bound_x_max;
  double bound_y_min, bound_y_max;

  RT1NavParam()
  {
    origin_x = 0;
    origin_y = 0;
    bound_x_min = -100;
    bound_x_max = 100;
    bound_y_min = -100;
    bound_y_max = 100; 
  }

  bool load(const ros::NodeHandle &nh)
  {
    nh.param<double>("origin_x", origin_x, origin_x);
    nh.param<double>("origin_y", origin_y, origin_y);
    nh.param<double>("bound_x_min", bound_x_min, bound_x_min);
    nh.param<double>("bound_x_max", bound_x_max, bound_x_max);
    nh.param<double>("bound_y_min", bound_y_min, bound_y_min);
    nh.param<double>("bound_y_max", bound_y_max, bound_y_max);


    return true;
  }
};

#endif 
