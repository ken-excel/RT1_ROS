#ifndef _find_loc_H_
#define _find_loc_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include "ros_start/PointRss.h"
#include "ros_start/RssAvg.h"
#include <string.h>

using namespace std;

ifstream dbFile("./db/database_rssSelective.txt");
ofstream outputFile;
string mode;
string criteria;

class locationFind
{
public:
	locationFind();
	void readDatabase(istream &input);

private:
	//ROS
	ros::NodeHandle nh_;
	ros::Subscriber rss_sub_avg_;
	ros::Subscriber rss_sub_loc_;

	ros_start::PointRss rss_comp_;

	typedef struct{
		string loc;
		vector<string> ssid;
		vector<double> rss;
		double rms;
		int count = 0;
	}database;

	vector<database> db;
	double min_rss = 90.0; //-90dB
	double min_dist = 30.0;

	//Functions
	void rssRead(const ros_start::RssAvg &rss);
	double rssComp(database rss_db, ros_start::RssAvg rss_in);	
	void rssStaticRead(const ros_start::PointRss &rss);
	double rssStaticComp(database rss_db, ros_start::PointRss rss_in);	
};

#endif
