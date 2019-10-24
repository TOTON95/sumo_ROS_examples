//Coded by Alexis Guijarro
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>

#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>

#include <stdio.h>
#include <cmath>
#include <math.h>
#include <iostream>

#define PI 3.141592653589793238462

double heading;                                                         //Drone's heading
int btn_emergency;                                                      //Emergency's Button to stop routine
geometry_msgs::Twist cmd_vel_sumo;                                     //Command Message

double ce_hdg,ce_pos_X,ce_pos_Y,ce_alt;                                 //Control efforts
std_msgs::Float64 st_pos_X,st_pos_Y,st_alt,st_hdg;                      //State variables
std_msgs::Float64 stp_pos_X,stp_pos_Y,stp_alt,stp_hdg;                  //Setpoint variables

struct v_object                                                         //Structure that describes the properties of the object
{
	double _posX,_posY,_posZ;                                       //Position of the drone
	double _errorX,_errorY,_errorZ;                                 //Error of the position of the drone
	double _orX,_orY,_orZ,_orW;                                     //Orientation of the drone
	double _roll,_pitch,_yaw,_yawRAD;                               //Roll,Pitch,Yaw (degrees), Yaw (radians)
	double _cmdX,_cmdY,_cmdZ,_cmdYAW;                               //Command values
	double rot_cmd_x,rot_cmd_y;                                     //Position in the rotated matrix
	double _velX,_velY,_velZ,_velYAW;                               //Velocities
	double abs_x,abs_y;                                             //Absolute position in X and Y
	double angle_res,angle_arc;                                     //Angle resultant, angle of the arc
}sumo;

struct Point                                                            //Structure to describe the Waypoints
{
	double x;
	double y;
};

void getJoyState(const sensor_msgs::Joy::ConstPtr& js)                  //Function to obtain the data from the emergency button of the joystick
{
	btn_emergency = js->buttons[0];
}

void getSumoPos(const geometry_msgs::TransformStamped::ConstPtr& pos)  //Function to obtain the position from the vicon system
{
	sumo._posX = pos->transform.translation.x;                     //Position in X
	sumo._posY = pos->transform.translation.y;                     //Position in Y
	sumo._posZ = pos->transform.translation.z;                     //Position in Z
	sumo._orX = pos->transform.rotation.x;                         //Rotation in X
	sumo._orY = pos->transform.rotation.y;                         //Rotation in Y
	sumo._orZ = pos->transform.rotation.z;                         //Rotation in Z
	sumo._orW = pos->transform.rotation.w;                         //Rotation in W

	tf::Quaternion q(pos->transform.rotation.x,pos->transform.rotation.y,pos->transform.rotation.z,pos->transform.rotation.w);
	tf::Matrix3x3 m(q);
	m.getRPY(sumo._roll,sumo._pitch,sumo._yawRAD);               //Get the Roll, Pitch, Yaw (Radians)
	sumo._yaw = sumo._yawRAD*(180/PI);                            //Convert the Yaw (Radians) into Yaw (Degrees)
	heading = sumo._yaw;                                           //Set the heading of the drone


	sumo.abs_x = sumo._posX;                                      //Set the absolute position of the drone in X
	sumo.abs_y = sumo._posY;                                      //Set the absolute position of the drone in Y
	//std::cout<<sumo._yawRAD<<std::endl;
}

double GetAngleDifference(double from, double to)
{
	double difference = to - from;
	while (difference < -180) difference += 360;
	while (difference > 180) difference -= 360;
	return difference;
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"sumo_vicon");                                     //Initiates the node
	ros::NodeHandle n;                                                      //Creates the node handler
	ros::Subscriber joy_sub,sumo_sub;                                     //Creates the subscribers of the joystick and the vicon system
	ros::Publisher cmd_vel_pub_sumo;                                       //Creates the publisher of the movement command
	joy_sub = n.subscribe("/joy",1000,getJoyState);                         //Initiates the Joystick Subscriber

	sumo_sub = n.subscribe("/vicon/SUMO_11_5_2018/SUMO_11_5_2018",1000,getSumoPos);
	cmd_vel_pub_sumo = n.advertise<geometry_msgs::Twist>("/rossumo1/cmd_vel_norm",1000);   //Initiates the Command publisher into  the topics

	//Init velocities
	cmd_vel_sumo.linear.x = 0;
	cmd_vel_sumo.angular.z = 0;

	ros::spinOnce();                                                        //Refresh the topics
	double hdg_target = 0;

	ros::Rate r(100);

	while(n.ok())                                                           //Execute the program until <Ctrl + C> is pressed
	{

		if(btn_emergency)                                               //If the joystick's button is pressed land the drone and finish the program
		{
			ros::Duration(0.525).sleep();
			printf("\n========== S T O P [ J S ]==========\n");
			cmd_vel_sumo.linear.x = 0;
			cmd_vel_sumo.angular.z = 0;
			cmd_vel_pub_sumo.publish(cmd_vel_sumo);
			break;
		}

		//where to go?
		double dest_x = 1.0;
		double dest_y = 1.0;

		//Find angle and distance
		double distance = 0;
		double angle = 0;

		distance = sqrt(pow(dest_x-sumo._posX,2)+pow(dest_y-sumo._posY,2));
		angle = atan2((dest_y - sumo._posY),(dest_x - sumo._posX));

		//face waypoint
		cmd_vel_sumo.angular.z = (angle-heading);
		cmd_vel_pub_sumo.publish(cmd_vel_sumo);

		//Stop
		cmd_vel_sumo.angular.z = 0;
		cmd_vel_pub_sumo.publish(cmd_vel_sumo);

		//Move to target
		cmd_vel_sumo.linear.x = distance;
		cmd_vel_pub_sumo.publish(cmd_vel_sumo);
	}
}
