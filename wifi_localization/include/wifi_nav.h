#ifndef _RT1_NAV_H_
#define _Rt1_NAV_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <rt1nav_param.h>
#include "wifi_nav/RssAvg.h"
#include "wifi_nav/Service.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <vector>
#include <iostream>
#include <fstream>

using namespace std;

ofstream outputFile, outputFile2;

class RT1Nav
{
public:
	RT1Nav();
	void process();
	void shutdown();
	move_base_msgs::MoveBaseGoal goal;

	ros::Time begin_time;

private:
	ros::NodeHandle nh_;
	RT1NavParam param_;
	ros::ServiceServer  pos_srv_; //move to goal order
	ros::Subscriber rss_goal_sub_; //subscribe rss data
	ros::Subscriber rss_robot_sub_; //subscribe rss data
	//ros::Subscriber status_sub_; //subscribe status of robot whether has it reached the goal yet?  

	wifi_nav::RssAvg rss_goal, rss_robot;
	
	double th;

	int instance = 0;
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

	void record(double data1, double data2);
	double compare(wifi_nav::RssAvg rss_g, wifi_nav::RssAvg rss_r);
	void rssRead_goal(const wifi_nav::RssAvg &rss);
	void rssRead_robot(const wifi_nav::RssAvg &rss);
	bool PosSrv(wifi_nav::Service::Request &req, wifi_nav::Service::Response &res);

	//Algorithm Function
	void blindwalk();
	void randomguess();
};

#endif
