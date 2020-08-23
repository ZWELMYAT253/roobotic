#include <ros/ros.h>
#include <actionlib_msgs/GoalID.h>
#include <actionlib_msgs/GoalStatus.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
//#include <visualization_msgs/Marker.h>
#include <math.h>
#include <stdio.h>

#define pi 3.1417

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base;

static geometry_msgs::Quaternion createQuaternionFromRPY(double roll, double pitch, double yaw);

double degreeToRadian(int degree);

int main(int argc,char** argv)
{
	ros::init(argc,argv,"nav_test");
	ros::NodeHandle n;
	if(argc != 3){
		ROS_INFO("rosrun chefbot_slam send_goal x y");
		return -1;
	}
	float goal_x = atoll(argv[1]);
	float goal_y = atoll(argv[2]);
	float x_ex  = 0.5;
	float y_ex  = 0.5;

	int total_wps =3;
	geometry_msgs::Quaternion quaternions[total_wps];
	double euler_angles[total_wps]={
		0,
		degreeToRadian(90),
		0
	};

	for(int i=0;i<total_wps;i++)
	{
		quaternions[i]=createQuaternionFromRPY(0,0,euler_angles[i]);
	}

	geometry_msgs::Pose waypoints[3];

	waypoints[0].position.x= goal_x + x_ex;
	waypoints[0].position.y= -goal_y + y_ex;       //left goal_y - y_ex;
	waypoints[0].position.z= 0.0;
	waypoints[0].orientation.x= quaternions[0].x;
	waypoints[0].orientation.y= quaternions[0].y;
	waypoints[0].orientation.z= quaternions[0].z;
	waypoints[0].orientation.w= quaternions[0].w;


//	waypoints[1].position.x= goal_x + (x_ex/2);
//	waypoints[1].position.y= -goal_y + (y_ex/2);  // left +
//	waypoints[1].position.z= 0.0;
//	waypoints[1].orientation.x= quaternions[1].x;
//	waypoints[1].orientation.y= quaternions[1].y;
//	waypoints[1].orientation.z= quaternions[1].z;
//	waypoints[1].orientation.w= quaternions[1].w;

	waypoints[1].position.x= goal_x;
	waypoints[1].position.y= -goal_y;             // left
	waypoints[1].position.z= 0.0;
	waypoints[1].orientation.x= quaternions[1].x;
	waypoints[1].orientation.y= quaternions[1].y;
	waypoints[1].orientation.z= quaternions[1].z;
	waypoints[1].orientation.w= quaternions[1].w;

	waypoints[2].position.x= goal_x;
	waypoints[2].position.y= -goal_y;  // left
	waypoints[2].position.z= 0.0;
	waypoints[2].orientation.x= quaternions[2].x;
	waypoints[2].orientation.y= quaternions[2].y;
	waypoints[2].orientation.z= quaternions[2].z;
	waypoints[2].orientation.w= quaternions[2].w;

	move_base move_base("move_base",true);
	move_base.waitForServer(ros::Duration(60));

	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id="map";
		

		for (int i=0;i<total_wps;i++)
		{
			goal.target_pose.header.stamp=ros::Time::now();
			goal.target_pose.pose=waypoints[i];
			move_base.sendGoal(goal);

			bool status=move_base.waitForResult(ros::Duration(60));
			if(status){
				ROS_INFO_STREAM("waypoints "<< i << "..parking");
			}
			else{
				move_base.cancelGoal();
				ROS_INFO("FAIL");
			}
		}
	
	ros::shutdown();
	}

static geometry_msgs::Quaternion createQuaternionFromRPY(double roll, double pitch, double yaw) 
{   
    geometry_msgs::Quaternion q;
    double t0 = cos(yaw * 0.5);
    double t1 = sin(yaw * 0.5);
    double t2 = cos(roll * 0.5);
    double t3 = sin(roll * 0.5);
    double t4 = cos(pitch * 0.5);
    double t5 = sin(pitch * 0.5);
    q.w = t0 * t2 * t4 + t1 * t3 * t5;
    q.x = t0 * t3 * t4 - t1 * t2 * t5;
    q.y = t0 * t2 * t5 + t1 * t3 * t4;
    q.z = t1 * t2 * t4 - t0 * t3 * t5;
    return q;
}

double degreeToRadian(int degree)
{
	return (degree/180.0)* 3.1415;
}
