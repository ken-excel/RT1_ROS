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
	rss_goal_sub_ = nh_.subscribe("/rss_goal_avg", 1, &RT1Nav::rssRead_goal, this);
	rss_robot_sub_ = nh_.subscribe("/rss_robot_avg", 1, &RT1Nav::rssRead_robot, this);
	//Set Initial position
	goal.target_pose.header.frame_id = "base_link";
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose.position.x= 0.0;
	goal.target_pose.pose.position.y= 0.0;
	goal.target_pose.pose.orientation.w= 1.0;
}



double RT1Nav::compare(wifi_nav::RssAvg rss_g, wifi_nav::RssAvg rss_r, double limit) //HERE
{
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
	double pos_error = sqrt(pow(poseAMCLx-goalx,2)+pow(poseAMCLy-goaly,2));
	outputFile<<elapsed<<","<<pos_error<<",";
	for (int i=0; i<rss_g.rss.size(); i++) 
	{
	bool matching = false; //!!!!!!!!reinitialize every loop
		if(rss_g.rss[i].x>-60 && list.find(rss_g.rss[i].name.c_str())!=list.end()){
			int j;
			for(j=0; j<rss_r.rss.size(); j++)
			{
				if(strcmp(rss_r.rss[j].name.c_str(),rss_g.rss[i].name.c_str()) == 0 && rss_r.rss[j].x>limit){
					matching = true;
					break;
				} 
			}
			
			if (matching)
			{
				d = fabs(rss_g.rss[i].x - rss_r.rss[j].x);
				cout << rss_g.rss[i].name << "|" << rss_g.rss[i].x << "|" << rss_r.rss[j].x << "|" << d << endl;
				root = pow(d,2);
				outputFile<<d<<","<<rss_g.rss[i].x<<","<<rss_r.rss[j].x<<",";
				if(strcmp(rss_r.rss[j].name.c_str(),"(AirPort10223)") == 0){
					if(rss_r.rss[j].x > -40) goal_reached = true;				
				}			
			}
			else
			{
				d = fabs(rss_g.rss[i].x - limit);
				cout << rss_g.rss[i].name << "|" << rss_g.rss[i].x << "|" << limit << "[min_rss]|" << d << endl;
				root = pow(d,2);
				outputFile<<d<<","<<rss_g.rss[i].x<<","<<"NotFound"<<",";					
			}
			ap_err.name=rss_g.rss[i].name.c_str();
			ap_err.diff=d;
			error.push_back(ap_err);
			sum += root;
			count++;
		}
		else outputFile<<"nil,-,-,"; 
	}
	ap_error = error;
	double avg;
	if (count!= 0) avg = sum/count;
	else avg = 0;
	rms = sqrt(avg);
	outputFile<<rms<<endl;
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

void calculateGoal(move_base_msgs::MoveBaseGoal &goal, double a, double b, double c){
	//Straight
	goal.target_pose.pose.position.x = a; //0.5
	goal.target_pose.pose.position.y = b; 
	//Turning
	double radians = c * (M_PI/180);
	tf::Quaternion quaternion; 
	quaternion = tf::createQuaternionFromYaw(radians);

	geometry_msgs::Quaternion qMsg;
	tf::quaternionTFToMsg(quaternion, qMsg);

	goal.target_pose.pose.orientation = qMsg;
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

void RT1Nav::Read_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgAMCL)
{
	poseAMCLx = msgAMCL->pose.pose.position.x;
	poseAMCLy = msgAMCL->pose.pose.position.y;
}

void RT1Nav::process()
{
  	//Scan&Compare
	ROS_INFO("SCAN");
  	rms_all = RT1Nav::compare(rss_goal, rss_robot, -60); 
  	cout << rms_all << endl;
  	//Register to log
  	if(goal_reached)cout<<"goal reached"<<endl;//goal
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
  		
		/*RMS Method*/
  		goal.target_pose.header.stamp = ros::Time::now();

  		if (instance>0) {
  			double drms = log[instance].rms - log[instance-1].rms;	
  			if (drms > 0){
  				//TURNING
  				calculateGoal(goal,0,0,0);//(goal,0,-1,-90);
  			} //wrong
  			else{
  				//STRAIGHT
 				calculateGoal(goal,1.5,0,0);
  			} //correct
  		}
  		else{
  			//STRAIGHT
 			calculateGoal(goal,1.5,0,0);
  		}

		/*Individual AP method
		if (instance>0) {
			int converge = 0, diverge = 0;  			
			for(int i; i<log[instance].error.size(); i++){			
				double temp = log[instance].error[i].diff - log[instance-1].error[i].diff;	
				if (temp>=0) converge++;
				else diverge++;			
			}
			if (converge<2){
  				//TURNING
  				calculateGoal(goal,0,-1,-90);
  			} //wrong
  			else{
  				//STRAIGHT
 				calculateGoal(goal,1.5,0,0);
  			} //correct
  		}
  		else{
  			//STRAIGHT
 			calculateGoal(goal,1.5,0,0);
  		}*/

  		instance++;
  		goal_ready = true; //send new goal
		ROS_INFO("GOAL_SEND");
  	}  
}

void RT1Nav::shutdown()
{
	outputFile<<"Time,Distance,";
	for(int i=0; i<rss_goal.rss.size(); i++) outputFile<<rss_goal.rss[i].name.c_str()<<",Goal,Robot,"; //each AP
	outputFile<<"RMS";  
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "rt1_nav_node");
	RT1Nav object;
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
	while(!ac.waitForServer(ros::Duration(5.0))){
    	ROS_INFO("Waiting for the move_base action server to come up");
  	}
  	ROS_INFO("Move_base action ready");
	ac.cancelAllGoals(); //Clear Old goals
	ros::Rate rate(50);
	object.begin_time = ros::Time::now();
	outputFile.open("experiment.txt");
	//outputFile2.open("rms.txt");
	while(ros::ok() && !interrupted)
	{
		signal(SIGINT, mySigintHandler);
		ros::spinOnce();
		if(rss_g_ready&&rss_r_ready){
		object.process();
		//LOOP
		if(goal_ready){
			//Move
			cout<<object.goal<<endl;
			ac.sendGoal(object.goal);

			//Stop
			ac.waitForResult(ros::Duration(20));
			if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
				ROS_INFO("Position reached");
				goal_ready = false;
			}
	  		else{
	  			ROS_INFO("ERROR");
	  			goal_ready = false;
				ac.cancelAllGoals();
	  		}
	  	}
		ROS_INFO("SLEEP");
		ros::Duration(4).sleep();
		}		
		rate.sleep();
	}

	if(interrupted){
	    ROS_ERROR("SHUTDOWN");
	    object.shutdown();
	    outputFile.close();
	    ac.cancelAllGoals();
	    //outputFile2.close();
	    ros::shutdown();
	}
	return 0;
}
