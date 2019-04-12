#include <signal.h>
#include "rt1_nav.h"

bool interrupted = false;

void mySigintHandler(int sig)
{
  interrupted = true;
}

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
//Set Move Base SimpleClientGoalState
MoveBaseClient ac("move_base", true);

RT1Nav::RT1Nav():
  nh_("~")
{
	param_.load(nh_);
	pos_srv_  = nh_.advertiseService("goal_pos", &RT1Nav::PosSrv, this);
	rss_goal_sub_ = nh_.subscribe("/rss_goal", 1, &RT1Nav::rssRead_goal, this);
	rss_robot_sub_ = nh_.subscribe("/rss_robot", 1, &RT1Nav::rssRead_robot, this);
	//pos_pub_  = nh_.advertise<move_base_msgs::MoveBaseGoal>("/amcl_pos", 1); //TODO
	//status_sub_ = nh_.subscribe("/amcl_status", 1000, &RT1Nav::Callback_status,this); //TODO

	//Set Initial position
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose.position.x= 0.0;
	goal.target_pose.pose.position.y= 0.0;
	goal.target_pose.pose.orientation.w= 1.0;

	goal_reach = false;
	goal_ready = false;
}

double RT1Nav::compare_all(ros_start::RssAvg rss_g, ros_start::RssAvg rss_r)
{
	bool matching = false;
	double root;
	double sum;
	double rms;
	int count;
	for (int i=0; i<AP_num; i++) 
	{
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
			root = pow(fabs(rss_g.rss[i].rss - rss_r.rss[j].rss),2);
		}
		else
		{
			root = pow(fabs(rss_g.rss[i].rss - min_rss),2);
		}
		sum += root;
		count++;
	}
	double avg = sum/count;
	rms = sqrt(avg);
	return rms;
}

void RT1Nav::compare_each(ros_start::RssAvg rss_g, ros_start::RssAvg rss_r) //HERE
{
	bool matching = false;
	vector<accesspoint> error;
	accesspoint ap_err;
	double d;
	for (int i=0; i<AP_num; i++) 
	{
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
			d = fabs(rss_g.rss[i].rss - rss_r.rss[j].rss);
		}
		else
		{
			d = fabs(rss_g.rss[i].rss - min_rss);
		}
		ap_err.name=rss_g.rss[i].name.c_str();
		ap_err.diff=d;
		error.push_back(ap_err);
	}
	ap_error = error;
}

bool RT1Nav::PosSrv(ros_start::Service::Request &req, ros_start::Service::Response &res)
{
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose.position.x=req.goal_x;
	goal.target_pose.pose.position.y=req.goal_y;
	goal.target_pose.pose.orientation.w= 1.0;
	goal_ready = true;
	ROS_INFO("Sending goal");
  	ac.sendGoal(goal);
	return true;
}

void RT1Nav::rssRead_goal(const ros_start::RssAvg &rss)
{
	rss_goal = rss;
}

void RT1Nav::rssRead_robot(const ros_start::RssAvg &rss)
{
	rss_robot = rss;
}

void RT1Nav::process()
{
	//LOOP
	if(goal_ready && !goal_reach){
	//Move
	ROS_INFO("Sending goal");
	ac.sendGoal(goal);

	//Stop
	ac.waitForResult();
	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		ROS_INFO("Position reached");
	}
  	else{
  		ROS_INFO("ERROR");
  		//exit or do something
  	}

  	//Scan&Compare
  	rms_all = RT1Nav::compare_all(rss_goal, rss_robot); //overall rms
  	RT1Nav::compare_each(rss_goal, rss_robot); //each AP's error

  	//Register to log
  	position_log plog;
  	plog.pos = instance;
  	plog.x = goal.target_pose.pose.position.x;
  	plog.y = goal.target_pose.pose.position.y;
  	plog.rms = rms_all;
  	plog.error = ap_error;
  	log.push_back(plog);

  	//Direction Guess Algorithm 
  	//RT1Nav::blindwalk();
  	
  	instance++;
  	goal_ready = false;
  	}
}

void RT1Nav::shutdown()
{

}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "rt1_nav_node");

  RT1Nav object;
  ros::Rate rate(50);
  while(ros::ok() && !interrupted)
  {
    signal(SIGINT, mySigintHandler);
    ros::spinOnce();
    object.process();
    rate.sleep();
  }

  if(interrupted)
  {
    ROS_ERROR("SHUTDOWN");
    object.shutdown();
    ros::shutdown();
  }

  return 0;
}
