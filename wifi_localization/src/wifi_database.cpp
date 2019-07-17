#include <ros/ros.h>
#include <ros/package.h>
#include "wifi_nav/RssAvg.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <signal.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <sqlite3.h> 

wifi_nav::RssAvg rss_robot;
bool interrupted = false;
bool rss_r_ready = false;
bool pose_ready = false;
double mapsize_x = 42; //m fullsize cell (partial cell counted as full cell) 
double mapsize_y = 20;	
double cellsize_x = 2;	
double cellsize_y = 2;	
int current_column = -1;
int current_row = -1;
double poseAMCLx, poseAMCLy;

static int callback(void *NotUsed, int argc, char **argv, char **azColName) {
   int i;
   for(i = 0; i<argc; i++) {
      printf("%s = %s\n", azColName[i], argv[i] ? argv[i] : "NULL");
   }
   printf("\n");
   return 0;
}

void mySigintHandler(int sig)
{
  interrupted = true;
}

void rssRead_robot(const wifi_nav::RssAvg &rss)
{
	rss_robot = rss;
	rss_r_ready = true;
}

void Read_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgAMCL)
{
	poseAMCLx = msgAMCL->pose.pose.position.x;
	poseAMCLy = msgAMCL->pose.pose.position.y;
	current_column = poseAMCLx/cellsize_x;
	current_row = -poseAMCLy/cellsize_y;
	pose_ready = true;
}

std::string updatequery(std::string ssid, double rssi, int column, int row)
{
	std::string sql;
	/* SQL statement Format*/
	std::string str_ssid = "'"+ssid+"'";
	std::ostringstream s_rssi, s_column, s_row;
	s_rssi << rssi;
	s_column << column;
	s_row << row;
	std::string str_rssi = s_rssi.str();
	std::string str_column = s_column.str(); 
	std::string str_row = s_row.str(); 

	sql = "INSERT OR REPLACE INTO RSSI_RECORD (SSID,COLUMN,ROW,RSSI)" \
   "VALUES ("+str_ssid+","+str_column+","+str_row+","+str_rssi+")";

   	return sql;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "rt1_nav_node");
	ros::NodeHandle nh_;

	ros::Subscriber pose_sub = nh_.subscribe("/amcl_pose", 10, &Read_pose);
	ros::Subscriber rss_robot_sub_ = nh_.subscribe("/rss_robot_avg", 10, &rssRead_robot);

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

	ros::Rate rate(5);

	while(ros::ok() && !interrupted)
	{
		signal(SIGINT, mySigintHandler);
		ROS_INFO("ROBOT_SCAN");
		if(rss_r_ready && pose_ready){
			for (int i=0; i<rss_robot.rss.size(); i++){
				//Update query data
				std::string ssid = rss_robot.rss[i].name;
				double rssi = rss_robot.rss[i].x;
				int column = current_column;
				int row = current_row;
				sql = updatequery(ssid, rssi, column, row);
				std::cout<<sql<<std::endl;
				/* Execute SQL statement */
			   	rc = sqlite3_exec(db, sql.c_str(), callback, 0, &zErrMsg);
			   	if( rc != SQLITE_OK ){
			    	fprintf(stderr, "SQL error: %s\n", zErrMsg);
			      	sqlite3_free(zErrMsg);
			   	} else {
			    	 fprintf(stdout, "Records created successfully\n");
			   	}
			}

			rss_r_ready = false;
			pose_ready = false;
		}
		ros::spinOnce();
		rate.sleep();
	}

	if(interrupted){
	    ROS_ERROR("SHUTDOWN");
	    ros::shutdown();
   		sqlite3_close(db);
	}
	return 0;
}
