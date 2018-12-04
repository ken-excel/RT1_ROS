#ifndef _wifi_node_H_
#define _wifi_node_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include "rss/RssData.h"
#include "ros_start/RssAvg.h"
#include "ros_start/RssDatumAvg.h"
#include <string.h>
#include <vector>

using namespace std;

// typedef struct{
// 		string name;
// 		int rss;
// }wifiData;

// wifiData Data[20];

ros::Time begin_time;
ofstream outputFile;
bool data_ready = false;

class wifiNode
{
public:
	wifiNode();
	void process();
	void shutdown();


private:
	//ROS
	ros::NodeHandle nh_;
	ros::Subscriber rss_sub_;
	ros::Publisher rss_pub_;

	ros_start::RssAvg rss_out_;
	ros_start::RssAvg db_out_;

	int	data_count = 0;

	//Functions
	void rssRead(const rss::RssData &rss);
	void rssRegis(string addr, float avg, int frequency);
};

#endif