/*
|
|	Author: Promsutipong Kengkij (k.promsutipong@srd.mech.tohoku.ac.jp)
| 	[1] Obstacle, slope, and step detection using angled laser range finder
|	[2] Obstacle Density Algorithm
| 
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "iostream"
#include "sensor_msgs/LaserScan.h"
//#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3Stamped.h"

#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)*M_PI/180)

double difficultyFront, difficultyLeft, difficultyRight = 0;
double dif[3];
//Customizable variables
double height = 0.4905;			//height of rplidar in meter;
double incline = 20; 		//inclination angle of rplidar in degree
double distance = height/tan(DEG2RAD(incline));			//Braking distance
double range = height/sin(DEG2RAD(incline)); 			//base value of range
//
std::string mode="move";

double difficulty_cal(float deg, float R, int mode){
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


void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) 
{
	difficultyFront=0;
	difficultyLeft=0;
	difficultyRight=0;
    int count = scan->scan_time / scan->time_increment;
    for(int i = 0; i < count; i++) {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        if(abs(degree)>159)difficultyFront += difficulty_cal(degree,scan->ranges[i],0);
        if(degree<-119&&degree>-161)difficultyLeft += difficulty_cal(degree,scan->ranges[i],-1);
        if(degree>119&&degree<161)difficultyRight += difficulty_cal(degree,scan->ranges[i],1);
    }
    //std::cout<<difficultyLeft<<" "<<difficultyFront<<" "<<difficultyRight<<std::endl;
    dif[0]=difficultyLeft;
    dif[1]=difficultyFront;
    dif[2]=difficultyRight;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "obsdetect");
    ros::NodeHandle n;
    ros::Publisher pub_difficulty = n.advertise<geometry_msgs::Vector3Stamped>("/rt1_con/difficulty", 10);
    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);
    geometry_msgs::Vector3Stamped diff;
    while(ros::ok())       
    {
		diff.vector.x=dif[0];
		diff.vector.y=dif[1];
		diff.vector.z=dif[2];
		diff.header.stamp=ros::Time::now();
    	pub_difficulty.publish(diff);
        /*if (mode != "move"){
            command.data = mode;
            std::cout<<mode<<std::endl;
            pub_mode.publish(command);
            mode = "move";
        }*/
        ros::spinOnce();
    }

    return 0;
}
