#include <ros/ros.h>
#include <ros/package.h>
#include "wifi_nav/RssAvg.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <signal.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <unordered_set>
#include <sqlite3.h> 
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <math.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Bool.h>

double lastx, lasty;
double d = 0;
bool first = true;

void Read_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgAMCL)
{
	double poseAMCLx = msgAMCL->pose.pose.position.x;
	double poseAMCLy = msgAMCL->pose.pose.position.y;
	if(first)
	{
		first = false;
	}
	else
	{
		d += sqrt(pow(poseAMCLx-lastx,2)+pow(poseAMCLy-lasty,2));
	}
	lastx = poseAMCLx;
	lasty = poseAMCLy;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "trajectory_distance_node");
	ros::NodeHandle nh_;
	ros::Rate rate(1000);
	ros::Subscriber pose_sub = nh_.subscribe("/amcl_pose", 10, &Read_pose);
	while(ros::ok())
	{
		std::cout<<d<<std::endl;
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}