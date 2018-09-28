//////////////////////////////////////////////////////////////////////////
//
//		[ROS] RT1 Handle Control
//
//		Subscribe: /rosrt_rt1
//		Publish: /rt1_con
//
//////////////////////////////////////////////////////////////////////////
#include "ros/ros.h"
#include "ros_start/Rt1Sensor.h"
#include <iostream>
#include <geometry_msgs/Twist.h>

double lin_vel, ang_vel;
double lin_handle, rot_handle; //handle data for pushing, rotating
double last_lin,last_rot; //last handle data for pushing, rotating
bool assist_on = false;

void handle_to_vel()
{
	//Direct Force => Vel
	lin_vel=lin_handle/20;
	ang_vel=rot_handle/200;
}

void Callback_sensor(const ros_start::Rt1Sensor &msg)
{
	if (msg.handle.force.x>2||msg.handle.force.x<-2){ 
		lin_handle=msg.handle.force.x;
		if (msg.handle.force.x>20) lin_handle = 20;
		else if (msg.handle.force.x<-20) lin_handle = -20;
		}
	else lin_handle = 0;
	if (msg.handle.torque.z>30||msg.handle.torque.z<-30){ 
		rot_handle = msg.handle.torque.z;
		if (msg.handle.torque.z>300) rot_handle = 300;
		else if (msg.handle.torque.z<-300) rot_handle = -300;
		}
	else rot_handle = 0;
	if (abs(lin_handle-last_lin)>5) lin_handle/=2;
	handle_to_vel();
	assist_on = true;
	last_lin = lin_handle;
	last_rot = rot_handle;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rt1handle");
    ros::NodeHandle n;
    geometry_msgs::Twist handle_vel;
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/rt1_con/vel", 10);
    ros::Subscriber sub_vel = n.subscribe("rosrt_rt1", 1000, Callback_sensor);
    while(ros::ok())
    {
    	if(assist_on) 
    		{
    			handle_vel.linear.x = lin_vel;
    			handle_vel.angular.z = ang_vel;
    			pub.publish(handle_vel);
        	}
        ros::spinOnce();
    }


    return 0;
}

