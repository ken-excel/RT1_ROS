#ifndef _find_loc_H_
#define _find_loc_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include "wifi_nav/PointRss.h"
#include "wifi_nav/RssAvg.h"
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

	wifi_nav::PointRss rss_comp_;

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
	void rssRead(const wifi_nav::RssAvg &rss);
	double rssComp(database rss_db, wifi_nav::RssAvg rss_in);	
	void rssStaticRead(const wifi_nav::PointRss &rss);
	double rssStaticComp(database rss_db, wifi_nav::PointRss rss_in);	
};

#endif
