#include <ros/ros.h>
#include "wifi_nav/RssAvg.h"
#include "wifi_nav/Service.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <math.h>
#include <iostream>
#include <fstream>

using namespace std;

bool pose_ready = false;
bool rss_g_ready = false;
bool rss_r_ready = false;

wifi_nav::RssAvg rss_goal, rss_robot;
double poseAMCLx, poseAMCLy;
double goalx = -3.53536679995;
double goaly = -4.66351709234;

int instance = 0;
double min_rss = -100.0;
double rms_all;


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void calculateGoal(move_base_msgs::MoveBaseGoal &goal, double a, double b, double c){
//Straight
  goal.target_pose.pose.position.x = a; //0.5
  goal.target_pose.pose.position.y = b;
  //Turning
  //double theta = c;
  //double radians = theta * (M_PI/180);
  //tf::Quaternion quaternion;
  //quaternion = tf::createQuaternionFromYaw(c); //radians

  //geometry_msgs::Quaternion qMsg;
  //tf::quaternionTFToMsg(quaternion, qMsg);

  //goal.target_pose.pose.orientation = qMsg;
  goal.target_pose.pose.orientation.z = -0.9237149596;
  goal.target_pose.pose.orientation.w = 0.383080505131;

}

void Read_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgAMCL)
{
	poseAMCLx = msgAMCL->pose.pose.position.x;
    poseAMCLy = msgAMCL->pose.pose.position.y;
    pose_ready = true;
}

void rssRead_goal(const wifi_nav::RssAvg &rss)
{
	rss_goal = rss;
	rss_g_ready = true;
}

void rssRead_robot(const wifi_nav::RssAvg &rss)
{
	rss_robot = rss;
	rss_r_ready = true;
}

double compare_limit(wifi_nav::RssAvg rss_g, wifi_nav::RssAvg rss_r, double limit) //HERE
{
	double d;
	double root;
	double sum;
	double rms;
	int count = 0;
	for (int i=0; i<rss_g.rss.size(); i++) 
	{
		bool matching = false;
		if(rss_g.rss[i].x>limit){
			int j;
			for(j=0; j<rss_r.rss.size(); j++)
			{
				if(strcmp(rss_r.rss[j].name.c_str(),rss_g.rss[i].name.c_str()) == 0){
					matching = true;
					break;
				} 
			}
			
			if (matching)
				{
					if(rss_r.rss[j].x>limit){	
						d = fabs(rss_g.rss[i].x - rss_r.rss[j].x);
						//cout << rss_g.rss[i].name << "|" << rss_g.rss[i].x << "|" << rss_r.rss[j].x << "|" << d << endl;
						root = pow(d,2);
					}
				}
				else
				{
					d = fabs(rss_g.rss[i].x - min_rss);
					//cout << rss_g.rss[i].name << "|" << rss_g.rss[i].x << "|" << min_rss << "[min_rss]|" << d << endl;
					root = pow(d,2);
				}
				sum += root;
				count++;
		}
	}
	double avg;
	if (count!= 0) avg = sum/count;
	else avg = 0;
	rms = sqrt(avg);
	return rms;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_snavigation_goals");
  ros::NodeHandle nh_;
  ros::Subscriber rss_goal_sub_ = nh_.subscribe("/rss_goal_avg", 1, &rssRead_goal);
  ros::Subscriber rss_robot_sub_ = nh_.subscribe("/rss_robot_avg", 1, &rssRead_robot);
  ros::Subscriber pose_sub_ = nh_.subscribe("/amcl_pose", 1, &Read_pose);
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  ROS_INFO("Ready");
  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  ros::Time begin_time = ros::Time::now();
  ofstream outputFile;
  outputFile.open("criteria_test.txt");

  ros::Rate rate(50);
  calculateGoal(goal,-3.53536679995,-4.66351709234,0); // x, y , quaternion z
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  outputFile << "Time,PosError,Criteria1,,Criteria2,,Criteria3,,Criteria4,,Criteria5,,Criretia6"<<endl;

  while(ros::ok())
  {
    ros::spinOnce();
  	double elapsed = (ros::Time::now() - begin_time).toSec();
	double pos_error = sqrt(pow(poseAMCLx-goalx,2)+pow(poseAMCLy-goaly,2));
	//cout << elapsed<<","<<pos_error<<",";
  	outputFile << elapsed<<","<<pos_error << ",";
  	//cout<<rss_g_ready<<rss_r_ready<<pose_ready<<endl;
  	if (rss_g_ready&&rss_r_ready&&pose_ready){
  		double array[6]={-50,-60,-70,-80,-90,-100}; //
  		for(double i :array){
  		rms_all = compare_limit(rss_goal, rss_robot, i); //left = static / right = moving / goal is com / robot is whill
  		cout << rms_all<<",";
  		outputFile << rms_all << ",";
  		if(rms_all > i/(-10)){
  			//not goal
  			cout<< "0,";
  			outputFile << "0,";
  		}
  		else{
  			//goal
  			cout<< "1,";
  			outputFile << "1,";
  		}
  		}
  	}
  	cout<<endl;
  	outputFile<<endl;

   //  calculateGoal(goal,0.5,0,0);

   // //ROS_INFO("Sending goal");
   //  ac.sendGoal(goal);

   //  ac.waitForResult(ros::Duration(30.0));

   //  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
   //    {
   //    	//ROS_INFO("Success");
   //    }
   //  else
   //    {
   //      //ROS_INFO("Fail");
   //      ac.cancelAllGoals();
   //    }
      rate.sleep();
	}
  outputFile.close();
  return 0;
}
