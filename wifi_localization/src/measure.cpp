#include <ros/ros.h>
#include "wifi_nav/RssAvg.h"
#include "wifi_nav/Service.h"
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <math.h>
#include <iostream>
#include <fstream>

using namespace std;

bool rss_g_ready = false;
bool rss_r_ready = false;

wifi_nav::RssAvg rss_goal, rss_robot;

int instance = 0;
double rms_all;

ofstream outputFile;

void rssRead_goal(const wifi_nav::RssAvg &rss)
{
	rss_goal = rss;
	rss_g_ready = true;
}

void rssRead_robot(const wifi_nav::RssAvg &rss)
{
	rss_robot = rss;
	rss_r_ready = true;
}

double compare_limit(wifi_nav::RssAvg rss_g, wifi_nav::RssAvg rss_r, double limit) //HERE
{
	double d;
	double root;
	double sum;
	double rms;
	int count = 0;
	for (int i=0; i<rss_g.rss.size(); i++) 
	{
		bool matching = false;
		//cout<<rss_g.rss[i].name.c_str()<<"|| ";
		//if(strcmp(rss_g.rss[i].name.c_str(),"(IsolationX)") == 0){ //Only Goal
		if(rss_g.rss[i].x>limit){ //lower limit
			int j;
			for(j=0; j<rss_r.rss.size(); j++)
			{
				if(strcmp(rss_r.rss[j].name.c_str(),rss_g.rss[i].name.c_str()) == 0 && rss_r.rss[j].x>limit){
					matching = true;
					break;
				} 
			}
			
			if (matching)
			{
				d = fabs(rss_g.rss[i].x - rss_r.rss[j].x);
				//cout << rss_g.rss[i].name << "|" << rss_g.rss[i].x << "|" << rss_r.rss[j].x << "|" << d << endl;
				root = pow(d,2);
				outputFile<<d<<","<<rss_g.rss[i].x<<","<<rss_r.rss[j].x<<",";			
			}
			else
			{
				d = fabs(rss_g.rss[i].x - limit);
				//cout << rss_g.rss[i].name << "|" << rss_g.rss[i].x << "|" << min_rss << "[min_rss]|" << d << endl;
				root = pow(d,2);
				outputFile<<d<<","<<rss_g.rss[i].x<<","<<"NotFound"<<",";					
			}
			sum += root;
			count++;
		}
		else outputFile<<"nil,-,-,"; 
	}
	double avg;
	if (count!= 0) avg = sum/count;
	else avg = 0;
	rms = sqrt(avg);
	return rms;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_snavigation_goals");
  ros::NodeHandle nh_;
  ros::Subscriber rss_goal_sub_ = nh_.subscribe("/rss_goal_avg", 1, &rssRead_goal);
  ros::Subscriber rss_robot_sub_ = nh_.subscribe("/rss_robot_avg", 1, &rssRead_robot);
  //tell the action client that we want to spin a thread by default

  ros::Time begin_time = ros::Time::now();
  outputFile.open("wifi_monitor.txt");

  ros::Rate rate(5);
  int pos = 0;
  while(ros::ok())
  {
    ros::spinOnce();
  	double elapsed = (ros::Time::now() - begin_time).toSec();
  	outputFile << elapsed<<",";
  	//cout<<rss_g_ready<<rss_r_ready<<pose_ready<<endl;
  	if (rss_g_ready&&rss_r_ready){
  		//double array[6]={-50,-60,-70,-80,-90,-100}; 
		double array[1]={-60}; 
  		for(double i :array){
  		rms_all = compare_limit(rss_goal, rss_robot, i); //left = static / right = moving / goal is com / robot is whill
		cout<<rms_all;		
		outputFile << rms_all;  		
		}
  	}
  	cout<<endl;
  	outputFile<<endl;
    	rate.sleep();
    }
    outputFile<<"Time,";
  for (int i=0; i<rss_goal.rss.size(); i++) outputFile<<rss_goal.rss[i].name.c_str()<<",Goal,Robot,"; //each AP
	outputFile<<"RMS";  
	outputFile.close();
  return 0;
}
