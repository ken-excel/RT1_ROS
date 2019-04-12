#ifndef _RT1_PARAM_H_
#define _RT1_PARAM_H_

#include <math.h>

class RT1Param
{
public:
  double max_lin, max_rot, min_lin, min_rot;
  double k_lin, k_rot;
  //+++NEW: max change for handle_to_vel

  RT1Param()
  {
    k_lin = 1;
    k_rot = 0.01;
    min_lin = 0.5;
    min_rot = 0.05;
    max_lin = 30;
    max_rot = 3.5;
    // k_lin = 0.05;
    // k_rot = 0.00667;
    // min_lin = 0.1;
    // min_rot = 0.2;
    // max_lin = 1.5;
    // max_rot = 2;
  }

  bool load(const ros::NodeHandle &nh)
  {
    nh.param<double>("k_lin", k_lin, k_lin);
    nh.param<double>("k_rot", k_rot, k_rot);
    nh.param<double>("min_lin", min_lin, min_lin);
    nh.param<double>("min_rot", min_rot, min_rot);
    nh.param<double>("max_lin", max_lin, max_lin);
    nh.param<double>("max_rot", max_rot, max_rot);


    return true;
  }
};

#endif // _RT1_PARAM_H_
