//////////////////////////////////////////////////////////////////////////
//
//		[ROS] RT1 cmd_vel control center
//
//		Subscribe: /rt1_con
//		Publish: /cmd_vel
//
//////////////////////////////////////////////////////////////////////////
#include "rt1.h"

RT1::RT1():
  nh_("~")
{
  param_.load(nh_);
  //set_mode_srv_  = nh_.advertiseService("set_mode", &RT1::SetModeSrv, this);
  obstacle_sub_ = nh_.subscribe("/rt1_con/difficulty", 1, &RT1::Callback_difficulty, this);
  cmd_vel_pub_  = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  sensor_sub_ = nh_.subscribe("/rosrt_rt1", 1000, &RT1::Callback_sensor,this);
  vel_lin = param_.vel_lin;
  vel_ang = param_.vel_ang;
}

geometry_msgs::Twist RT1::velocity_compute()
{
    geometry_msgs::Twist computed;
    computed.linear.x = vel_lin;
    computed.angular.z = vel_ang;
    if(difF>20){
        if(computed.linear.x>0)computed.linear.x = 0;
        else if(computed.linear.x<0)computed.linear.x *=1.3; //Easier Back 
        }
    else{
        if(computed.linear.x>0)computed.linear.x*=fabs((difF-20)/20);
    }
    if(computed.angular.z>0){
    //Left
        if(difL>20){
            computed.angular.z=0;
        }
        else computed.angular.z*=fabs((difL-20)/20);
    }
    else if(computed.angular.z<0){
    //Right
        if(difR>20){
            computed.angular.z=0;
        }
        else computed.angular.z*fabs((difR-20)/20);
    }

    return computed;
}

void RT1::Callback_sensor(const ros_start::Rt1Sensor &msg)
{
    double lin_handle, rot_handle; //handle data for pushing, rotating
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
    if (abs(lin_handle-last_lin)>5) lin_handle/=2; //check THIS back and forth prevention
    //+++NEW: Set maximum change per loop => act like acceleration : set in Param !!!Take Care that this could prevent fast stop!
    //Adjust value
    vel_lin=lin_handle/20;
    vel_ang=rot_handle/150;
    last_lin = lin_handle;
    last_rot = rot_handle;
    command_move=true;
}

void RT1::Callback_difficulty(const geometry_msgs::Vector3Stamped &diff)
{
    difL=diff.vector.x;
    difF=diff.vector.y;
    difR=diff.vector.z;
}

void RT1::process()
{
    if (command_move){
        output_cmd_vel_ = velocity_compute();
        cmd_vel_pub_.publish(output_cmd_vel_);
        command_move=false;
    }
}

void RT1::shutdown()
{
    output_cmd_vel_.angular.z = 0;
    output_cmd_vel_.linear.x = 0; 
    cmd_vel_pub_.publish(output_cmd_vel_);
}