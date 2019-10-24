#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <stdlib.h>     
#include <time.h>
#include <iostream>
#include <vector>

using namespace std;

ros::Subscriber ctl;
geometry_msgs::Twist v;
ros::Publisher _cmd_vel_pub;

void messageCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
	v.linear.x = msg->axes[1];
	v.angular.z = msg->axes[0];
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "sumo_manual_ctl_node");
	ros::NodeHandle nh_public, nh_private("~");

	//Subscribers
	ctl = nh_public.subscribe("/joy",100,messageCallback);
	//Publisher
	_cmd_vel_pub = nh_public.advertise<geometry_msgs::Twist>("/rossumo1/cmd_vel_norm", 1);

	ros::Rate r(100);

	while(nh_public.ok())
	{
		_cmd_vel_pub.publish(v);
		ros::spinOnce();
		r.sleep(); 
	} 
	
	v.linear.x = 0;
	v.angular.z = 0;
	_cmd_vel_pub.publish(v);
  	return 0;
}
