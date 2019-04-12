/*
 * Copyright (c) 2014, RoboPeak
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 *    this list of conditions and the following disclaimer in the documentation 
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR 
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*
 *  RoboPeak LIDAR System
 *  RPlidar ROS Node client test app
 *
 *  Copyright 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 * 
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "iostream"
#include "sensor_msgs/LaserScan.h"
#define RAD2DEG(x) ((x)*180./M_PI)

std::string mode="move";

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int count = scan->scan_time / scan->time_increment;
    for(int i = 0; i < count; i++) {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        //if (degree<0.5&&degree>-0.5) ROS_INFO(": [%f, %f]", degree, scan->ranges[i]);
        if ((degree<=-175||degree>=175)&&scan->ranges[i]<=0.7&&scan->ranges[i]>=0.1) {
        mode="stop"; //0.5 = 50 cm approx
        }
        else if ((degree>=120&&degree<=175)&&scan->ranges[i]<=0.6&&scan->ranges[i]>=0.1){
            if(mode=="move") mode="left"; //turn left obsta right
            else if(mode=="right") mode="stop";
        } 
        else if ((degree<=-120&&degree>=-175)&&scan->ranges[i]<=0.6&&scan->ranges[i]>= 0.1){
            if(mode=="move") mode="right"; //turn right obsta left
            else if(mode=="left") mode="stop";
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obsdetect");
    ros::NodeHandle n;
    //Stop command
    std_msgs::String command;
    command.data = "move";
    ros::Publisher pub_mode = n.advertise<std_msgs::String>("/rt1_con/mode", 10);
    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);
    while(ros::ok())       
    {   
        if (mode != "move"){
            command.data = mode;
            std::cout<<mode<<std::endl;
            pub_mode.publish(command);
            mode = "move";
        }
        ros::spinOnce();
    }

    return 0;
}
