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

wifi_nav::RssAvg rss_goal;
bool interrupted = false;
bool rss_g_ready = false;
double mapsize_x = 42; //m fullsize cell (partial cell counted as full cell) 
double mapsize_y = 20;	
double cellsize_x = 2;	
double cellsize_y = 2;	
std::unordered_set<std::string> whitelist;
double error[200];
int cb_index;
int goal_column, goal_row;
double goalx, goaly;

static int callback(void *NotUsed, int argc, char **argv, char **azColName) {
	double x = atof(argv[3]);
   error[cb_index] +=  pow(x,2);
   // for(i = 0; i<argc; i++) {
   //    printf("%s = %s\n", azColName[i], argv[i] ? argv[i] : "NULL");
   // } 
   //printf("\n");
   
   cb_index++;
   return 0;
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
	ros::init(argc, argv, "rt1_nav_node");
	ros::NodeHandle nh_;

	ros::Subscriber rss_goal_sub_ = nh_.subscribe("/rss_goal_avg", 10, &rssRead_goal);
	ros::Publisher goal_pub_ = nh_.advertise<move_base_msgs::MoveBaseGoal>("wifi_goal", 20);

	whitelist = {"(AirPort10223)","(haptic)","(HirataLab)","(HirataLab_Guest)"};

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
		ROS_INFO("GOAL_SCAN");
		if(rss_g_ready){
			for (int i=0; i<rss_goal.rss.size(); i++) 
			{
				if(rss_goal.rss[i].x>-60 && whitelist.find(rss_goal.rss[i].name.c_str())!=whitelist.end()){
				//Update query data
					cb_index = 0;
					std::string ssid = "'"+rss_goal.rss[i].name;+"'";
					sql = "SELECT * FROM RSSI_RECORD WHERE SSID ="+ssid+" ORDER BY row ASC, column ASC";

					/* Execute SQL statement */
				   	rc = sqlite3_exec(db, sql.c_str(), callback, 0, &zErrMsg);
				   	if( rc != SQLITE_OK ){
				    	fprintf(stderr, "SQL error: %s\n", zErrMsg);
				      	sqlite3_free(zErrMsg);
				   	} else {
				    	 fprintf(stdout, "Records created successfully\n");
				   	}
				}
			}
			int low;
			double lowest_error = 9999;
			for (int i=0;i<200;i++){
				error[i] = sqrt(error[i])/whitelist.size(); //rootmeansqure error
				if (error[i]<lowest_error){
					lowest_error = error[i];
					low = i;
				}
			}
			goal_column = (low)%int(mapsize_x/cellsize_x);
			goal_row = (low)/(mapsize_x/cellsize_x);
			goalx = (goal_column+0.5)*cellsize_x; //goal coor
			goaly = (goal_row+0.5)*cellsize_y; //goal coor
			//send goal or publish goal
			move_base_msgs::MoveBaseGoal goal;
			goal.target_pose.header.frame_id = "map";
			goal.target_pose.header.stamp = ros::Time::now();
			goal.target_pose.pose.position.x= goalx;
			goal.target_pose.pose.position.y= goaly;
			goal.target_pose.pose.orientation.w= 1.0;

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

