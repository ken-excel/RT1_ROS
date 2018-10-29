#ifndef _ANG_COM_H_
#define _ANG_COM_H_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "iostream"
#include "sensor_msgs/LaserScan.h"
#include "rplidar_param.h"

class RPLidar
{
public:
	RPLidar();
	void process();
	void shutdown();

private:
	ros::NodeHandle nh_;
	ros::Publisher  difficulty_pub_ ; //cmd vel
	ros::Subscriber laserScan_sub_; //sensor sub
	RT1Param param_;

	double difL,difF,difR;
	double dif[3];
	double height, incline, distance, range;
	//geometry_msgs::Vector3Stamped diff;
	rplidar_ros::Difficulty diff;

	double difficulty_cal(float deg, float R, int mode);
	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
};

#endif
