//algorithm.cpp
#include "rt1_nav.h"
#include "math.h"

void RT1Nav::blindwalk()
{
	double dist;
	double th;
	if(log[instance].rms < 5)
	{
		//goal reached
		goal_reach = true;
	}
	else
	{
		if(log.size()>1) //except first loop
		{
			double drms = fabs(log[instance].rms - log[instance-1].rms)/fabs(log[1].rms-log[0].rms);

			if(log[instance].rms<=log[instance-1].rms)
			{
				drms > 1 ? dist = 2 : dist = 2*drms;
				th += 0;
			}
			else
			{
				drms > 1 ? dist = 2 : dist = 2*drms;
				th += M_PI/3;
				//rotate 90
				//move forward
			}
		}
		else //first loop
		{
			dist = 2;
			th += 0;
			//move forward;
		}
		th = th%(2*M_PI); //Keep angle in the range of 0-360
		goal.target_pose.pose.position.x += dist*cos(th); //here
		goal.target_pose.pose.position.y += dist*sin(th);
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
		goal.target_pose.orientation = odom_quat;
		goal_ready = true;
	}
}

void RT1Nav::blindwalkAP()
{
	double dist;
	double th;
	int converge = 0;
	int diverge = 0;
	int reach = 0;
	for(i=0; i<AP_num; i++){
		if(log[instance].error[AP_num].diff < 5) reach++;
		else
		{
			if (log[instance].error[AP_num].diff > log[instance-1].error[AP_num].diff) diverge++;
			else if(log[instance].error[AP_num].diff < log[instance-1].error[AP_num].diff) converge++;
		}
	}
	int correct = converge+reach;
	if (reach >= 9)
	{
		//goal reached
		goal_reach = true;
	}
	else
	{
		if(log.size()>1) //except first loop
		{
			
			if(correct >= 6)
			{
				(correct/AP_num) > 1 ? dist = 2 : dist = 2-1*(correct/AP_num);
				th += 0;
			}
			else
			{
				(correct/AP_num) > 1 ? dist = 2 : dist = 2-1*(correct/AP_num);
				th += M_PI/3;
				//rotate 90
				//move forward
			}
		}
		else //first loop
		{
			dist = 2;
			th += 0;
			//move forward;
		}
		goal.target_pose.pose.position.x += dist*cos(th); //here
		goal.target_pose.pose.position.y += dist*sin(th);
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
		goal.target_pose.orientation = odom_quat;
		goal_ready = true;
	}
}

void RT1Nav::randomguess()
{
	goal_ready = true;	
}