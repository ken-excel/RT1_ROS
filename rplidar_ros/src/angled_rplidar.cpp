/*
|
|	Author: Promsutipong Kengkij (k.promsutipong@srd.mech.tohoku.ac.jp)
| 	[1] Obstacle, slope, and step detection using angled laser range finder
|	[2] Obstacle Density Algorithm
| 
*/

#include "angled_rplidar.h"

#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)*M_PI/180)

RPLidar::RPLidar():
  nh_("~")
{
  param_.load(nh_);
  difficulty_pub_ = nh_.advertise<rplidar_ros::Difficulty>("/rt1_con/difficulty", 10);
  laserScan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 1000, &RPLidar::scanCallback,this);
  height = param_.height;
  incline = param_.incline;
  distance = param_.distance;
  range = param_.range;
  diff.difficulty.data.resize(3);
}


double RPLidar::difficulty_cal(float deg, float R, int mode){
	double difficulty;
	double sweeping = 180-abs(deg);				//Sweeping angle from the front
	double base_value = range/cos(DEG2RAD(sweeping));	//Base value for range at any sweeping angles
	double base_angle =	mode*140;				//0 for front, -1 for left, +1 for right
	double theta;
	if (mode!=0) theta = abs(base_angle-deg);
	else theta = sweeping;
	if(mode==0){
		if (0.8*base_value<=R && R<=1.2*base_value) difficulty=0.0;	//Floor
		else if (R<0.8*base_value&&R>0.1){					//Obstacle or Wall
			if (theta<=10) difficulty=2; 			//Upfront (Theta means angle difference from base angle in each mode)
			else if (theta>10&&theta<=20) difficulty=0.5;
		}						
		else if (R>1.2*base_value && R < 100){					//Step
			if (theta<=10) difficulty=2; //Upfront
			else if (theta>10&&theta<=20) difficulty=0.5;
		}		
		else difficulty = 0;
	}
	if(mode!=0){
		if (0.8*base_value<=R && R<=1.2*base_value) difficulty=0.0;	//Floor
		else if (R<0.8*base_value&&R>0.1){					//Obstacle or Wall
			if (theta<=10) difficulty=1; 			//Upfront (Theta means angle difference from base angle in each mode)
		}						
		else if (R>1.2*base_value && R < 100){					//Step
			if (theta<=10) difficulty=1; //Upfront
		}		
		else difficulty = 0;		
	}
	return difficulty;
}

void RPLidar::scanCompensator(const sensor_msgs::LaserScan::ConstPtr& scan) 
{
	//Assign scan data to position
}

void RPLidar::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) 
{
	RPLidar::scanCompensator(scan);
	difF=0;
	difL=0;
	difR=0;
    int count = scan->scan_time / scan->time_increment;
    for(int i = 0; i < count; i++) {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        if(abs(degree)>159)difF += difficulty_cal(degree,scan->ranges[i],0);
        if(degree<-119&&degree>-161)difL += difficulty_cal(degree,scan->ranges[i],-1);
        if(degree>119&&degree<161)difR += difficulty_cal(degree,scan->ranges[i],1);
    }
    //std::cout<<difL<<" "<<difF<<" "<<difR<<std::endl;
    dif[0]=difL;
    dif[1]=difF;
    dif[2]=difR;
}

void RPLidar::process()
{
	// diff.vector.x=dif[0];
	// diff.vector.y=dif[1];
	// diff.vector.z=dif[2];
	diff.difficulty.data[0] = dif[0];
	diff.difficulty.data[1] = dif[1];
	diff.difficulty.data[2] = dif[2];
	std::cout<<"difL"<<dif[0]<<"difF"<<dif[1]<<"difR"<<dif[2]<<std::endl;
	diff.header.stamp=ros::Time::now();
    difficulty_pub_.publish(diff);
}

void RPLidar::shutdown()
{
	// diff.vector.x=0;
	// diff.vector.y=0;
	// diff.vector.z=0;
	diff.difficulty.data[0] = dif[0];
	diff.difficulty.data[1] = dif[1];
	diff.difficulty.data[2] = dif[2];
	diff.header.stamp=ros::Time::now();
    difficulty_pub_.publish(diff);
}