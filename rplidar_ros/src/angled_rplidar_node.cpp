#include <ros/ros.h>
#include <ros/package.h>
#include <signal.h>
#include "angled_rplidar.h"

bool interrupted = false;

void mySigintHandler(int sig)
{
  interrupted = true;
}
int main(int argc, char** argv)
{

  ros::init(argc, argv, "rplidar_node");

  RPLidar object;
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
    ros::shutdown();
    object.shutdown();
  }

  return 0;
}
