#include "include_wifi_nav.h"

bool interrupted = false;
bool goal_ready = false;
bool rss_g_ready = false;
bool rss_r_ready = false;
bool front_clear = false;
bool right_clear = false;
bool left_clear = false;

actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);


void mySigintHandler(int sig)
{
  interrupted = true;
}

double compare(wifi_nav::RssAvg rss_g, wifi_nav::RssAvg rss_r, double limit) //HERE
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
		if(rss_g.rss[i].x>-60 && whitelist.find(rss_g.rss[i].name.c_str())!=whitelist.end()){
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

void genPoint(double pos_x, double pos_y)
{
	geometry_msgs::Point p;
    	p.x = pos_x;
    	p.y = pos_y;
      	p.z = 0;

    points.points.push_back(p);
}


void calculateGoal(double a, double b, double c){
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
	goal.target_pose.header.stamp = ros::Time::now();
	ac.sendGoal(goal);
	ac.waitForResult(ros::Duration(20));
	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		ROS_INFO("Position reached");
	}
	else{
	 	ROS_INFO("ERROR");
		ac.cancelAllGoals();
	}
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

void Read_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgAMCL)
{
	poseAMCLx = msgAMCL->pose.pose.position.x;
	poseAMCLy = msgAMCL->pose.pose.position.y;
}

void wall_check(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	front_clear = right_clear = left_clear = true;
    int count = scan->scan_time / scan->time_increment;
    for(int i = 0; i < count; i++) {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        if (degree>-100 && degree < -80 && scan->ranges[i] < 3) right_clear = false;
        else if (degree>-10 && degree < 10 && scan->ranges[i] < 3) front_clear = false;
        else if (degree>80 && degree < 100 && scan->ranges[i] < 3) left_clear = false;
        ROS_INFO(": [%f, %f]", degree, scan->ranges[i]);
    }
}

bool condition_check()
{
	if (instance>1) 
	{
		double drms = log_history[instance].rms - log_history[instance-1].rms;	
		if (drms > 0){
			return false;
		} //wrong
		else{
			//STRAIGHT
			return true;
		} //correct
	}
}


bool FindGoal(double a, double b, double c)
{
	ros::spinOnce();
	bool goal_check = false;
	cout<<goal<<endl;
	calculateGoal(a,b,c);
	ROS_INFO("SCAN");
  	rms_all = compare(rss_goal, rss_robot, -60); 
  	cout << rms_all << endl;
  	
  	//Register to log 
  	position_log plog;
  	plog.pos = instance;	
  	plog.x = poseAMCLx;
  	plog.y = poseAMCLy;
  	plog.rms = rms_all;
  	plog.error = ap_error;
  	log_history.push_back(plog);
  	instance++;
  	genPoint(plog.x,plog.y);
  	marker_pub.publish(points);

  	if(goal_reached){
  		cout<<"goal reached - finish"<<endl;
  		return true;
  	}
  	else{
  		if(condition_check) cout<<"correct - continue"<<endl;
  		else{
  			cout<<"wrong - return"<<endl;
  			return false;
  		}
  	}

	if(front_clear && !goal_check) goal_check = FindGoal(1,0,0);
	if(right_clear && !goal_check) goal_check = FindGoal(0,-1,-90);
	if(left_clear && !goal_check) goal_check = FindGoal(0,1,90);
	return goal_check;
}

void shutdown()
{
	outputFile<<"Time,Distance,";
	for(int i=0; i<rss_goal.rss.size(); i++) outputFile<<rss_goal.rss[i].name.c_str()<<",Goal,Robot,"; //each AP
	outputFile<<"RMS";  
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "rt1_nav_node");
	ros::NodeHandle nh_;

	ros::Subscriber rss_goal_sub_ = nh_.subscribe("/rss_goal_avg", 10, &rssRead_goal);
	ros::Subscriber rss_robot_sub_ = nh_.subscribe("/rss_robot_avg", 10, &rssRead_robot);
	ros::Subscriber laser_sub = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 1000, &wall_check);
	marker_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 20);

	//Set Initial position
	goal.target_pose.header.frame_id = "base_link";
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose.position.x= 0.0;
	goal.target_pose.pose.position.y= 0.0;
	goal.target_pose.pose.orientation.w= 1.0;

	//point format
	points.header.frame_id = "/map";
    points.header.stamp = ros::Time::now();
    points.ns = "points_history";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;
    points.scale.x = 0.2;
    points.scale.y = 0.2;
    points.color.r = 1.0;
    points.color.a = 1.0;

    //Movebase
	while(!ac.waitForServer()){
    	ROS_INFO("Waiting for the move_base action server to come up");
  	}
  	ROS_INFO("Move_base action ready");
	ac.cancelAllGoals(); //Clear Old goals

	begin_time = ros::Time::now();
	outputFile.open("experiment.txt");
	//outputFile2.open("rms.txt");
	//while(ros::ok() && !interrupted)
	//{
		//signal(SIGINT, mySigintHandler);
		ROS_INFO("SCAN");
		ros::spinOnce();
  		rms_all = compare(rss_goal, rss_robot, -60); 
  		cout << rms_all << endl;
	  	
	  	//Register to log 
	  	position_log plog;
	  	plog.pos = instance;	
	  	plog.x = poseAMCLx;
	  	plog.y = poseAMCLy;
	  	plog.rms = rms_all;
	  	plog.error = ap_error;
	  	log_history.push_back(plog);
	  	instance++;
	  	genPoint(plog.x,plog.y);
	  	marker_pub.publish(points);
		FindGoal(1,0,0);

	//}

	if(interrupted){
	    ROS_ERROR("SHUTDOWN");
	    shutdown();
	    outputFile.close();
	    ac.cancelAllGoals();
	    //outputFile2.close();
	    ros::shutdown();
	}
	return 0;
}
