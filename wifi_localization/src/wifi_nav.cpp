#include <signal.h>
#include <tf/transform_datatypes.h>
#include "wifi_nav.h"

bool interrupted = false;
bool goal_ready = false;
bool rss_g_ready = false;
bool rss_r_ready = false;

void mySigintHandler(int sig)
{
  interrupted = true;
}

RT1Nav::RT1Nav():
  nh_("~")
{
	param_.load(nh_);
	pos_srv_  = nh_.advertiseService("move_pos", &RT1Nav::PosSrv, this);
	rss_goal_sub_ = nh_.subscribe("/rss_pc_avg", 1, &RT1Nav::rssRead_goal, this);
	rss_robot_sub_ = nh_.subscribe("/rss_robot_avg", 1, &RT1Nav::rssRead_robot, this);
	//Set Initial position
	goal.target_pose.header.frame_id = "base_link";
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose.position.x= 0.0;
	goal.target_pose.pose.position.y= 0.0;
	goal.target_pose.pose.orientation.w= 1.0;
}

void RT1Nav::record(double data1, double data2)
{
	outputFile<<data1<<","<<data2<<",";
}

double RT1Nav::compare(wifi_nav::RssAvg rss_g, wifi_nav::RssAvg rss_r) //HERE
{
	bool matching = false;
	vector<accesspoint> error;
	accesspoint ap_err;
	double d;
	double root;
	double sum;
	double rms;
	int count = 0;

	//Write to Log
	ros::Time t = ros::Time::now();
	double elapsed = (t - begin_time).toSec();
	outputFile<<elapsed<<",";
	outputFile2<<elapsed<<",";

	for (int i=0; i<rss_g.rss.size(); i++) 
	{
		if(rss_g.rss[i].x>-60){
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
					if(rss_r.rss[j].x>-60){	
						RT1Nav::record(rss_g.rss[i].x,rss_r.rss[j].x);
						d = fabs(rss_g.rss[i].x - rss_r.rss[j].x);
						cout << rss_g.rss[i].name << "|" << rss_g.rss[i].x << "|" << rss_r.rss[j].x << "|" << d << endl;
						root = pow(d,2);
					}
				}
				else
				{
					RT1Nav::record(rss_g.rss[i].x,min_rss);
					d = fabs(rss_g.rss[i].x - min_rss);
					root = pow(d,2);
				}
				ap_err.name=rss_g.rss[i].name.c_str();
				ap_err.diff=d;
				error.push_back(ap_err);
				sum += root;
				count++;
		}
	}
	outputFile<<endl;
	ap_error = error;
	double avg = sum/count;
	rms = sqrt(avg);
	cout<<rms<<endl<<endl;
	outputFile2<<rms<<endl;
	return rms;
}

bool RT1Nav::PosSrv(wifi_nav::Service::Request &req, wifi_nav::Service::Response &res)
{
	ROS_INFO("REQUESTED");
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose.position.x=req.goal_x;
	goal.target_pose.pose.position.y=req.goal_y;
	goal.target_pose.pose.orientation.w= 1.0;
	goal_ready = true;
	return true;
}

void RT1Nav::rssRead_goal(const wifi_nav::RssAvg &rss)
{
	rss_goal = rss;
	rss_g_ready = true;
}

void RT1Nav::rssRead_robot(const wifi_nav::RssAvg &rss)
{
	rss_robot = rss;
	rss_r_ready = true;
}

void RT1Nav::process()
{
  	//Scan&Compare
  	if (rss_g_ready&&rss_r_ready){
  		rms_all = RT1Nav::compare(rss_goal, rss_robot); //left = static / right = moving / goal is com / robot is whill
  		//cout << rms_all << endl;
  	}
  	//Register to log
  	if(fabs(rms_all )<3)cout<<"goal reached";//goal
  	else{ //goal not reach
  	  	if(!goal_ready) //log current position if robot is stopping
  	  	{	
  		  	position_log plog;
  		  	plog.pos = instance;
  		  	plog.x = goal.target_pose.pose.position.x;
  		  	plog.y = goal.target_pose.pose.position.y;
  		  	plog.rms = rms_all;
  		  	plog.error = ap_error;
  		  	log.push_back(plog);
  	 	}
  		//goal generation
  		
  		goal.target_pose.header.stamp = ros::Time::now();

  		if (instance>0) {
  			double drms = log[instance].rms - log[instance-1].rms;	
  			if (drms > 0){
  				//TURNING
  				goal.target_pose.pose.position.x = 0.3; 
				double theta = 30; 
				double radians = theta * (M_PI/180);
				tf::Quaternion quaternion;
				quaternion = tf::createQuaternionFromYaw(radians);
				geometry_msgs::Quaternion qMsg;
				tf::quaternionTFToMsg(quaternion, qMsg);
				goal.target_pose.pose.orientation = qMsg;
  			} //wrong
  			else{
  				//STRAIGHT
 				goal.target_pose.pose.position.x = 0.5; 
 				goal.target_pose.pose.orientation.w = 1.0;
  			} //correct
  		}
  		else{
  			//STRAIGHT
 			goal.target_pose.pose.position.x = 0.5; 
 			goal.target_pose.pose.orientation.w = 1.0;
  		}

  		goal_ready = true; //send new goal
  	}  

  	//Direction Guess Algorithm 
  	//RT1Nav::blindwalk();
  	
 //  	goal.target_pose.header.frame_id = "map";
	// goal.target_pose.header.stamp = ros::Time::now();
	// goal.target_pose.pose.position.x=1;
	// goal.target_pose.pose.position.y=1;
	// goal.target_pose.pose.orientation.w= 1.0;
	// goal_ready = true;

 //  	instance++;
}

void RT1Nav::shutdown()
{
	outputFile<<"Time,";
	for (int i = 0; i < rss_robot.rss.size(); i++){
		outputFile<<rss_robot.rss[i].name<<"ST,";
		outputFile<<rss_robot.rss[i].name<<"MV";
		if (i != rss_robot.rss.size()-1) outputFile<<",";
		else outputFile<<endl;
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "rt1_nav_node");
	RT1Nav object;
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>  ac("move_base", true);
	ros::Rate rate(50);
	object.begin_time = ros::Time::now();
	outputFile.open("rss_kalman_comparison.txt");
	outputFile2.open("rms.txt");
	while(ros::ok() && !interrupted)
	{
		signal(SIGINT, mySigintHandler);
		ros::spinOnce();
		object.process();
		//LOOP
		if(goal_ready){
			//Move
			ROS_INFO("Sending goal");
			ac.sendGoal(object.goal);

			//Stop
			ac.waitForResult();
			if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
				ROS_INFO("Position reached");
				goal_ready = false;
			}
	  		else{
	  			ROS_INFO("ERROR");
	  			//exit or do something
	  		}
	  	}
		rate.sleep();
	}

	if(interrupted){
	    ROS_ERROR("SHUTDOWN");
	    object.shutdown();
	    outputFile.close();
	    outputFile2.close();
	    ros::shutdown();
	}
	return 0;
}
