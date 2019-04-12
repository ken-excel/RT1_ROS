#ifndef _RT1_H_
#define _Rt1_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <rt1_param.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include "std_msgs/String.h"
#include <iostream>
#include <geometry_msgs/Twist.h>
#include "ros_start/Difficulty.h"
#include "ros_start/Rt1Sensor.h"
#include <geometry_msgs/Wrench.h>

class RT1
{
public:
	RT1();
	void process();
	void shutdown();

private:
	ros::NodeHandle nh_;
	ros::ServiceServer  set_mode_srv_; //in progress
	RT1Param param_;
	ros::Subscriber obstacle_sub_; //obstacle data - inprogress 
	ros::Publisher  cmd_vel_pub_ ; //cmd vel
	ros::Publisher  cmd_tor_pub_ ; //cmd tor
	ros::Subscriber sensor_sub_; //sensor sub
	nav_msgs::Odometry odom_;
  	geometry_msgs::Twist output_cmd_vel_; //need remap
  	geometry_msgs::Wrench output_cmd_tor_;

  	bool command_move;
	double difL, difF, difR;
	double vel_lin, vel_rot;
	double force, torque;
	double lin_handle, rot_handle, last_lin, last_rot; //last handle data for pushing, rotating
	double max_lin, max_rot, min_lin, min_rot;
	double k_lin, k_rot;

	geometry_msgs::Twist velocity_compute();
	void Callback_vel(const geometry_msgs::Twist &twist);
	void Callback_sensor(const ros_start::Rt1Sensor &msg);
	void Callback_difficulty(const ros_start::Difficulty &diff);
};

#endif
