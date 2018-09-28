#ifndef _RT1_PARAM_H_
#define _RT1_PARAM_H_

#include <math.h>

class RT1Param
{
public:
  double  vel_lin;
  double  vel_ang;
  //+++NEW: max change for handle_to_vel

  RT1Param()
  {
    vel_lin = 0;
    vel_ang = 0;
  }

  bool load(const ros::NodeHandle &nh)
  {
    nh.param<double>("vel_lin", vel_lin, vel_lin);
    nh.param<double>("vel_ang", vel_ang, vel_ang);

    return true;
  }
};

#endif // _RT1_PARAM_H_
