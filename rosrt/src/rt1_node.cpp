#include <ros/ros.h>
#include <ros/package.h>
#include <signal.h>
#include "rt1.h"

bool interrupted = false;

void mySigintHandler(int sig)
{
  interrupted = true;
}
int main(int argc, char** argv)
{

  ros::init(argc, argv, "rt1_node");

  RT1 object;
  ros::Rate rate(50);
  while(ros::ok() && !interrupted)
  {
    signal(SIGINT, mySigintHandler);
    ros::spinOnce();
    object.process();
    rate.sleep();
  }

  if(interrupted)
  {
    ROS_ERROR("SHUTDOWN");
    object.shutdown();
    ros::shutdown();
  }

  return 0;
}
