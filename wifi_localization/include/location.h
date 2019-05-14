#ifndef _location_node_H_
#define _location_node_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include "wifi_nav/RssAvg.h"
#include "wifi_nav/PointRss.h"
#include <string.h>
#include <vector>

using namespace std;

bool ready = false;
bool writing_finish = false;

FILE * pFile;
FILE * qFile;

string loc_name;
ros::Time begin_time;

class locationNode //Take wifi's RSS avg, order wifi id from RSS value (highest signal come first), define number of wifi used N, find each average and publish 
{
public:
	locationNode();
	void process();

private:
	//ROS
	ros::NodeHandle nh_;
	ros::Subscriber rss_avg_sub_;
	ros::Publisher rss_loc_pub_;

	unsigned int wifiMax = 10;
	unsigned int wifiCount;

	typedef struct{
		string name;
		long int sum_rss = 0;
		long int sum_dist = 0;
		unsigned int count = 0;
		float mean_rss;
		float med_rss;
		float mod_rss;
		float mean_dist;
		float med_dist;
		float mod_dist;
	}rssData;

	vector<rssData> rss_temp_;
	vector<rssData> rss_arr_;

	//Functions
	void rssRead(const wifi_nav::RssAvg &rss); //Read, Find Avg. Send Top 5 Candidates
	void findAverage(rssData &rss_in_); //find all average in location msg in this function
};

#endif
