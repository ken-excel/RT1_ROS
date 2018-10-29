#ifndef _wifi_node_H_
#define _wifi_node_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <vector>
#include <iostream>
#include "rss/RssData.h"
#include <string.h>

using namespace std;

typedef struct{
		string name;
		int rss;
}wifiData;

wifiData Data[20];
int	data_count = 0;

class wifiNode
{
public:
	wifiNode();

private:
	//ROS
	ros::NodeHandle nh_;
	ros::Subscriber rss_sub_;

	//Functions
	void rssRead(const rss::RssData &rss);
	void rssRegis(string addr, float avg);
};

#endif