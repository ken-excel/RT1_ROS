#ifndef _RT1_NAV_H_
#define _Rt1_NAV_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <rt1nav_param.h>
#include "ros_start/RssAvg.h"
#include "ros_start/Service.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <vector>
#include <iostream>

using namespace std;

class RT1Nav
{
public:
	RT1Nav();
	void process();
	void shutdown();
	move_base_msgs::MoveBaseGoal goal;

private:
	ros::NodeHandle nh_;
	RT1NavParam param_;
	ros::ServiceServer  pos_srv_; //move to goal order
	ros::Subscriber rss_goal_sub_; //subscribe rss data
	ros::Subscriber rss_robot_sub_; //subscribe rss data
	//ros::Subscriber status_sub_; //subscribe status of robot whether has it reached the goal yet?  

	ros_start::RssAvg rss_goal, rss_robot;
	
	double th;

	int instance = 0;
	int AP_num = 10;
	double min_rss = -90.0;
	double rms_all;

	typedef struct{
		string name;
		double diff;
	}accesspoint;

	vector<accesspoint> ap_error;

	typedef struct{
		int pos;
		double x,y;
		double rms;
		vector<accesspoint> error; //USAGE: log.error[0].name log.error[0].diff
	}position_log;

	vector<position_log> log;

	double compare_all(ros_start::RssAvg rss_g, ros_start::RssAvg rss_r);
	void compare_each(ros_start::RssAvg rss_g, ros_start::RssAvg rss_r);
	void rssRead_goal(const ros_start::RssAvg &rss);
	void rssRead_robot(const ros_start::RssAvg &rss);
	bool PosSrv(ros_start::Service::Request &req, ros_start::Service::Response &res);

	//Algorithm Function
	void blindwalk();
	void randomguess();
};

#endif
