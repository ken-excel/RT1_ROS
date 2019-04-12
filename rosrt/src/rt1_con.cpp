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
  cmd_tor_pub_ = nh_.advertise<geometry_msgs::Wrench>("/cmd_tor", 1);
  sensor_sub_ = nh_.subscribe("/rosrt_rt1", 1000, &RT1::Callback_sensor,this);
  max_lin = param_.max_lin;
  max_rot = param_.max_rot;
  min_lin = param_.min_lin;
  min_rot = param_.min_rot;
  k_lin = param_.k_lin;
  k_rot = param_.k_rot;
  difL = 0;
  difF = 0;
  difR = 0;
  last_lin = 0;
  last_rot = 0;
}

geometry_msgs::Twist RT1::velocity_compute()
{
    geometry_msgs::Twist computed;
    computed.linear.x = vel_lin;
    computed.angular.z = vel_rot;
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
        else computed.angular.z*=fabs((difR-20)/20);
    }
    return computed;
}

void RT1::Callback_sensor(const ros_start::Rt1Sensor &msg)
{
    lin_handle = msg.handle.force.x;
    rot_handle = msg.handle.torque.z; //handle data for pushing, rotating
    
    /* cmd_vel
    vel_lin=lin_handle*k_lin;
    vel_rot=rot_handle*k_rot;
    if (fabs(vel_lin) < min_lin){
        vel_lin = 0;
    }
    else if (vel_lin > max_lin) vel_lin = max_lin;
    else if (vel_lin < -max_lin) vel_lin = -max_lin;
    
    if (fabs(vel_rot) < min_rot){   
        vel_rot = 0;
    }
    else if (vel_rot > max_rot) vel_rot = max_rot;
    else if (vel_rot < -max_rot) vel_rot = -max_rot;

    if(fabs(vel_lin - last_lin) > (max_lin/4)){
        if(vel_lin > last_lin) vel_lin = last_lin + (max_lin/4);
        else vel_lin = last_lin - (max_lin/4);
    }

    if(fabs(vel_rot - last_rot) > (max_rot/4)){
        if(vel_rot > last_rot) vel_rot = last_rot + (max_rot/4);
        else if (vel_rot < last_rot) vel_rot = last_rot - (max_rot/4);
    }
    last_lin = vel_lin;
    last_rot = vel_rot;
    command_move=true;
    */

    //cmd_tor
    force=lin_handle*k_lin;
    torque=rot_handle*k_rot;
    if (fabs(force) < min_lin){
        force = 0;
    }
    else if (force > max_lin) force = max_lin;
    else if (force < -max_lin) force = -max_lin;
    
    if (fabs(vel_rot) < min_rot){   
        torque = 0;
    }
    else if (torque > max_rot) torque = max_rot;
    else if (torque < -max_rot) torque = -max_rot;

    // if(fabs(force - last_lin) > (max_lin/4)){
    //     if(force > last_lin) force = last_lin + (max_lin/4);
    //     else force = last_lin - (max_lin/4);
    // }

    // if(fabs(torque - last_rot) > (max_rot/4)){
    //     if(torque > last_rot) torque = last_rot + (max_rot/4);
    //     else if (torque < last_rot) torque = last_rot - (max_rot/4);
    // }
    // last_lin = force;
    // last_rot = torque;
    command_move=true;
}

void RT1::Callback_difficulty(const ros_start::Difficulty &diff)
{
    difL=diff.difficulty.data[0];
    difF=diff.difficulty.data[1];
    difR=diff.difficulty.data[2];
    std::cout<<difL<<" "<<difF<<" "<<difR<<std::endl;
}

void RT1::process()
{
    if (command_move){
        //output_cmd_vel_ = velocity_compute();
        //cmd_vel_pub_.publish(output_cmd_vel_);
        output_cmd_tor_.force.x = force;
        output_cmd_tor_.torque.z = torque;
        cmd_tor_pub_.publish(output_cmd_tor_);
        command_move=false;
    }
}

void RT1::shutdown()
{
    // output_cmd_vel_.angular.z = 0;
    // output_cmd_vel_.linear.x = 0; 
    // cmd_vel_pub_.publish(output_cmd_vel_);
    output_cmd_tor_.force.x = 0;
    output_cmd_tor_.torque.z = 0;
    cmd_tor_pub_.publish(output_cmd_tor_);
}