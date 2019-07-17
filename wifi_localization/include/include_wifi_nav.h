#ifndef _RT1_NAV_H_
#define _Rt1_NAV_H_

#include <ros/ros.h>
#include <ros/package.h>
#include "wifi_nav/RssAvg.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <signal.h>
#include <tf/transform_datatypes.h>
#include "sensor_msgs/LaserScan.h"
#include <vector>
#include <unordered_set>
#include <iostream>
#include <fstream>

using namespace std;

#define RAD2DEG(x) ((x)*180./M_PI)


ofstream outputFile, outputFile2;

move_base_msgs::MoveBaseGoal goal;
ros::Time begin_time;
ros::Publisher marker_pub; 

wifi_nav::RssAvg rss_goal, rss_robot;
visualization_msgs::Marker points;
	
double th;

int instance = 0;
double rms_all;

double poseAMCLx, poseAMCLy;
double goalx = -4.07508277893;
double goaly = -1.92194521427;

bool goal_reached = false;

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

vector<position_log> log_history;

unordered_set<string> whitelist = {"(AirPort10223)","(haptic)","(HirataLab)","(HirataLab_Guest)"};

void record(double data1, double data2);
double compare(wifi_nav::RssAvg rss_g, wifi_nav::RssAvg rss_r, double limit);
void rssRead_goal(const wifi_nav::RssAvg &rss);
void rssRead_robot(const wifi_nav::RssAvg &rss);
void wall_check(const sensor_msgs::LaserScan::ConstPtr& scan);
void Read_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgAMCL);
void genPoint(double pos_x, double pos_y);

//Algorithm Function
void blindwalk();
void randomguess();

#endif
