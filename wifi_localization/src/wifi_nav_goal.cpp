#include <ros/ros.h>
#include <ros/package.h>
#include "wifi_nav/RssAvg.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <signal.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <unordered_set>
#include <sqlite3.h> 
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <math.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Bool.h>

wifi_nav::RssAvg rss_goal;
bool interrupted = false;
bool rss_g_ready = false;
bool pose_ready = false;
bool db_cb = false;
double mapsize_x = 42; //m fullsize cell (partial cell counted as full cell) 
double mapsize_y = 20;	
double cellsize_x = 2;	
double cellsize_y = 2;	
std::unordered_set<std::string> whitelist;
double db_value;
double poseAMCLx, poseAMCLy;

static int callback(void *ptr, int argc, char **argv, char **azColName) {
	//Column 0 1 2 3 = SSID, ROW, COLUMN, RSSI
	db_value = argv[3] ? atof(argv[3]) : 9999;
	//std::cout<< db_value <<std::endl;
 	return 0;
}

void Read_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgAMCL)
{
	poseAMCLx = msgAMCL->pose.pose.position.x;
	poseAMCLy = msgAMCL->pose.pose.position.y;
	pose_ready = true;
}

void mySigintHandler(int sig)
{
  interrupted = true;
}

void rssRead_goal(const wifi_nav::RssAvg &rss)
{
	rss_goal = rss;
	rss_g_ready = true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "wifi_nav_goal_node");
	ros::NodeHandle nh_;

	ros::Subscriber pose_sub = nh_.subscribe("/amcl_pose", 10, &Read_pose);
	ros::Subscriber rss_goal_sub_ = nh_.subscribe("/rss_goal_avg", 10, &rssRead_goal);
	ros::Publisher goal_pub_ = nh_.advertise<move_base_msgs::MoveBaseGoal>("wifi_goal", 20);
	ros::Publisher db_signal_pub_ = nh_.advertise<std_msgs::Bool>("db_signal", 1);

	typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
	MoveBaseClient ac("move_base", true);
	while(!ac.waitForServer(ros::Duration(5.0))){
    		ROS_INFO("Waiting for the move_base action server to come up");
  	}
  	ROS_INFO("Ready");

	whitelist = {"(AirPort10223)","(haptic)","(HirataLab)","(WiFiNav_A)","(WiFiNav_B)","(WiFiNav_C)"};

	sqlite3 *db;
   	char *zErrMsg = 0;
   	int rc;

   	/* Open database */
	rc = sqlite3_open("heatmap.db", &db);

	if( rc ) {
    	fprintf(stderr, "Can't open database: %s\n", sqlite3_errmsg(db));
      	return(0);
   	} else {
    	fprintf(stderr, "Opened database successfully\n");
   	}

   	/* SQL statement Format*/
	std::string sql;

	while(ros::ok() && !interrupted)
	{
		signal(SIGINT, mySigintHandler);
		if(rss_g_ready){
			double lowest = 9999;
			double error = 0;
			int lowest_row, lowest_column;
			std_msgs::Bool db_signal;
			for (int i=0; i<(mapsize_y/cellsize_y); i++) 
			{
				//std::cout<<"Row: " << i;
				for (int j=0; j<(mapsize_x/cellsize_x); j++) 
				{
					error = 0;
					//std::cout<<" Column: " << j;	
					for(int k =0; k<rss_goal.rss.size(); k++){
					if(whitelist.find(rss_goal.rss[k].name)!=whitelist.end()){
						std::ostringstream s_column, s_row;
						s_column << j;
						s_row << i;
						std::string str_column = s_column.str(); 
						std::string str_row = s_row.str(); 
						std::string str_ssid = "'"+rss_goal.rss[k].name+"'";
						//std::cout << i << "," << j << "," << rss_goal.rss[k].name << std::endl;
						db_signal.data = true;
						db_signal_pub_.publish(db_signal);
						sql = "SELECT * FROM RSSI_RECORD WHERE ROW ="+str_row+" AND COLUMN ="+str_column+" AND SSID =" +str_ssid;
						std::cout << sql << std::endl;
						/* Execute SQL statement */
						db_value = 9999;
					  	rc = sqlite3_exec(db, sql.c_str(), callback, 0, &zErrMsg);
					   	if( rc != SQLITE_OK ){
					    		//fprintf(stderr, "SQL error: %s\n", zErrMsg);
					      		sqlite3_free(zErrMsg);
							db_cb = false;
					   	} else {
							db_cb = true;
					    	 //fprintf(stdout, "Records read successfully\n");
					   	}
					   	error += fabs(db_value-rss_goal.rss[k].x);
					}
				   	}
					db_signal.data = false;
					db_signal_pub_.publish(db_signal);
				   	error = error / whitelist.size();
					std::cout << error << std::endl;
				   	if(error<lowest){
				   		lowest = error;
						lowest_row = i;
						lowest_column = j;
					}
				}
			}
			if(db_cb){
				std::cout<<lowest_row<<","<<lowest_column<<std::endl;
				double goalx = (lowest_column+0.5)*cellsize_x; //goal coor
				double goaly = -(lowest_row+0.5)*cellsize_y; //goal coor
				//send goal or publish goal
		
				//Initialize Goal with direction facing toward center of a cell
				move_base_msgs::MoveBaseGoal goal;
				goal.target_pose.header.frame_id = "map";
				double radians = atan2(goaly-poseAMCLy,goalx-poseAMCLx);
				tf::Quaternion quaternion; 
				quaternion = tf::createQuaternionFromYaw(radians);
				geometry_msgs::Quaternion qMsg;
				tf::quaternionTFToMsg(quaternion, qMsg);
				goal.target_pose.pose.orientation = qMsg;

				//In place rotation
				goal.target_pose.header.stamp = ros::Time::now();
				goal.target_pose.pose.position.x= poseAMCLx;
				goal.target_pose.pose.position.y= poseAMCLy;
				goal_pub_.publish(goal);
				ROS_INFO("In-place rotation");
				ac.sendGoal(goal);
				ac.waitForResult();

				goal.target_pose.header.stamp = ros::Time::now();
				goal.target_pose.pose.position.x= goalx;
				goal.target_pose.pose.position.y= goaly;
				goal_pub_.publish(goal);
				ROS_INFO("Sending goal");
	  			ac.sendGoal(goal);
				ac.waitForResult();
	   			if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	   			{
	   				ROS_INFO("Success");
	  			}
	   			else
	   			{
	   				ROS_INFO("Fail");
	   				ac.cancelAllGoals();
	   			}
			}
			else std::cout<<"DBError"<<std::endl;
			rss_g_ready = false;
		}
		ros::spinOnce();
	}

	if(interrupted){
	    ROS_ERROR("SHUTDOWN");
	    ros::shutdown();
   		sqlite3_close(db);
	}
	return 0;
}

