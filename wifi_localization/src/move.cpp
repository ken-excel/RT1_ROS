#include "ros/ros.h"
#include "wifi_nav/Service.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_pos");
  if (argc != 3)
  {
    ROS_INFO("usage: move_pos x y (distance in m)");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<wifi_nav::Service>("move_pos");
  wifi_nav::Service srv;
  srv.request.goal_x = atoll(argv[1]);
  srv.request.goal_y = atoll(argv[2]);
  if (client.call(srv))
  {
    ROS_INFO("Goal sent");
  }
  else
  {
    ROS_ERROR("Failed to call service");
    return 1;
  }

  return 0;
}
