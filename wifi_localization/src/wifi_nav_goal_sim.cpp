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
int db_row, db_column;
double poseAMCLx, poseAMCLy;
double error = 0;
std::ofstream outputFile;

typedef struct{
	double value = 9999;
	int column = -1;
	int row = -1;
}knn;

static int callback(void *ptr, int argc, char **argv, char **azColName) {
	//Column 0 1 2 3 4 5 = SSID, ROW, COLUMN, RSSI, X, Y
	if (error == 9999) error = 0;
	std::string db_ssid = argv[0] ? argv[0] : "NONE";
	if(whitelist.find(db_ssid)!=whitelist.end()){
		//std::cout<<"WHITELISTED:" <<db_ssid<<std::endl;
		for(int k =0; k<rss_goal.rss.size(); k++){
			if (db_ssid == rss_goal.rss[k].name){
				db_value = argv[3] ? atof(argv[3]) : 9999;
				db_row = argv[1] ? atoi(argv[1]) : -1;
				db_column = argv[2] ? atoi(argv[2]) : -1;
				error += fabs(db_value-rss_goal.rss[k].x);
			}
		}
	}	
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

	std::string file_row, file_column,file_name;
	if(argc==3){
		file_row = argv[1];
		file_column = argv[2];
	} 
	file_name = "exp_result("+file_row+","+file_column+").txt";
	outputFile.open(file_name);

    ros::Rate rate(0.2);
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
			knn first_min, second_min, third_min, fourth_min, fifth_min;
			double lowest_row, lowest_column;
			std_msgs::Bool db_signal;
			for (double i=0; i<20; i=i+0.5) 
			{
				//std::cout<<"Row: " << i;
				for (double j=0; j<42; j=j+0.5) 
				{
					error = 9999;
					//std::cout<<" Column: " << j;	
					std::ostringstream s_x, s_y;
					s_x << j;
					s_y << i;
					std::string str_x = s_x.str(); 
					std::string str_y = s_y.str(); 
					//std::cout << i << "," << j << "," << rss_goal.rss[k].name << std::endl;
					db_signal.data = true;
					db_signal_pub_.publish(db_signal);
					sql = "SELECT * FROM RSSI_RECORD WHERE X ="+str_x+" AND Y ="+str_y;
					//std::cout << sql << std::endl;
					/* Execute SQL statement */
					 rc = sqlite3_exec(db, sql.c_str(), callback, 0, &zErrMsg);
					if( rc != SQLITE_OK ){
				    		//fprintf(stderr, "SQL error: %s\n", zErrMsg);
				      		sqlite3_free(zErrMsg);
						db_cb = false;
				   	} else {
						db_cb = true;
				    	 //fprintf(stdout, "Records read successfully\n");
				   	}
					db_signal.data = false;
					db_signal_pub_.publish(db_signal);
				   	error = error / whitelist.size();
					if (error != 1666.5) std::cout << db_row << "," << db_column << ":" << error << std::endl;
				   	if(error<first_min.value){
				   		fifth_min = fourth_min;
				   		fourth_min = third_min;
				   		third_min = second_min;
				   		second_min = first_min;
				   		first_min.value = error;
				   		first_min.row = db_row;
				   		first_min.column = db_column;
					}
					else if (error < second_min.value){
				   		fifth_min = fourth_min;
				   		fourth_min = third_min;
						third_min = second_min;
				   		second_min.value = error;
				   		second_min.row = db_row;
				   		second_min.column = db_column;
					}
					else if (error < third_min.value){
				   		fifth_min = fourth_min;
				   		fourth_min = third_min;
				   		third_min.value = error;
				   		third_min.row = db_row;
				   		third_min.column = db_column;
					}
					else if (error < fourth_min.value){
				   		fifth_min = fourth_min;
				   		fourth_min.value = error;
				   		fourth_min.row = db_row;
				   		fourth_min.column = db_column;
					}
					else if (error < fifth_min.value){
				   		fifth_min.value = error;
				   		fifth_min.row = db_row;
				   		fifth_min.column = db_column;
					}
				}
			}
			int knn_array[5] = {first_min.row*21+first_min.column,second_min.row*21+second_min.column,third_min.row*21+third_min.column,fourth_min.row*21+fourth_min.column, fifth_min.row*21+fifth_min.column};
			std::sort(knn_array, knn_array+5);
			int number = knn_array[0];
			int mode = number;
			int count = 1;
			int countMode = 1;

			for (int i=1; i<5; i++)
			{
			      if (knn_array[i] == number) 
			      { // count occurrences of the current number
			         ++count;
			      }
			      else
			      { // now this is a different number
			            if (count > countMode) 
			            {
			                  countMode = count; // mode is the biggest ocurrences
			                  mode = number;
			            }
			           count = 1; // reset count for the new number
			           number = knn_array[i];
			  }
			}
			lowest_row = mode/21;
			lowest_column = mode%21;
			if(db_cb){
				std::cout<<lowest_row<<","<<lowest_column<<std::endl;
				outputFile<<lowest_row<<","<<lowest_column<<std::endl;
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

				goal.target_pose.header.stamp = ros::Time::now();
				goal.target_pose.pose.position.x= goalx;
				goal.target_pose.pose.position.y= goaly;
				goal_pub_.publish(goal);
				ROS_INFO("Sending goal");
			}
			else std::cout<<"DBError"<<std::endl;
			rss_g_ready = false;
		}
		ros::spinOnce();
		rate.sleep();
	}

	if(interrupted){
	    ROS_ERROR("SHUTDOWN");
	    ros::shutdown();
   		sqlite3_close(db);
   		outputFile.close();
	}
	return 0;
}

