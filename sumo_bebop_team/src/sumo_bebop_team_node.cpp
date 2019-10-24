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

#include <sumo_bebop_team/ArcDrone.h>

#include <stdio.h>
#include <cmath>
#include <math.h>
#include <iostream>

#define PI 3.141592653589793238462

double heading;								//Drone's heading
int btn_emergency;							//Emergency's Button to stop routine
geometry_msgs::Twist cmd_vel_bebop;					//Command Message 

double ce_hdg,ce_pos_X,ce_pos_Y,ce_alt;					//Control efforts 
std_msgs::Float64 st_pos_X,st_pos_Y,st_alt,st_hdg;			//State variables
std_msgs::Float64 stp_pos_X,stp_pos_Y,stp_alt,stp_hdg;			//Setpoint variables

double sumo_offset_x = 0.0;							//Sumo's Offset in X
double sumo_offset_y = 0.0;							//Sumo's Offset in Y

struct v_object								//Structure that describes the properties of the object
{
	double _posX,_posY,_posZ;					//Position of the drone
	double _errorX,_errorY,_errorZ;					//Error of the position of the drone
	double _orX,_orY,_orZ,_orW;					//Orientation of the drone
	double _roll,_pitch,_yaw,_yawRAD;				//Roll,Pitch,Yaw (degrees), Yaw (radians)
	double _cmdX,_cmdY,_cmdZ,_cmdYAW;				//Command values 
	double rot_cmd_x,rot_cmd_y;					//Position in the rotated matrix
	double _velX,_velY,_velZ,_velYAW;				//Velocities
	double abs_x,abs_y;						//Absolute position in X and Y
	double angle_res,angle_arc;					//Angle resultant, angle of the arc
}bebop,sumo;

struct Point								//Structure to describe the Waypoints
{
	double x;
	double y;
};

void getJoyState(const sensor_msgs::Joy::ConstPtr& js)			//Function to obtain the data from the emergency button of the joystick
{
	btn_emergency = js->buttons[0];
}

void getBebopPos(const geometry_msgs::TransformStamped::ConstPtr& pos)	//Function to obtain the position from the vicon system
{
	bebop._posX = pos->transform.translation.x;			//Position in X
	bebop._posY = pos->transform.translation.y;			//Position in Y
	bebop._posZ = pos->transform.translation.z;			//Position in Z
	bebop._orX = pos->transform.rotation.x;				//Rotation in X
	bebop._orY = pos->transform.rotation.y;				//Rotation in Y
	bebop._orZ = pos->transform.rotation.z;				//Rotation in Z
	bebop._orW = pos->transform.rotation.w;				//Rotation in W

	tf::Quaternion q(pos->transform.rotation.x,pos->transform.rotation.y,pos->transform.rotation.z,pos->transform.rotation.w);
	tf::Matrix3x3 m(q);
	m.getRPY(bebop._roll,bebop._pitch,bebop._yawRAD);		//Get the Roll, Pitch, Yaw (Radians)
	bebop._yaw = bebop._yawRAD*(180/PI);				//Convert the Yaw (Radians) into Yaw (Degrees)
	heading = bebop._yaw;						//Set the heading of the drone	
	
		
	bebop.abs_x = bebop._posX;					//Set the absolute position of the drone in X
	bebop.abs_y = bebop._posY;					//Set the absolute position of the drone in Y
	//std::cout<<bebop._yawRAD<<std::endl;
}

void getSumoPos(const geometry_msgs::TransformStamped::ConstPtr& pos)		//Function to obtain the position of the sumo from the vicon system
{
	sumo._posX = pos->transform.translation.x;				//Position in X
	sumo._posY = pos->transform.translation.y;				//Position in Y
	sumo._posZ = pos->transform.translation.z;				//Position in Z
	sumo._orX = pos->transform.rotation.x;					//Rotation in X
	sumo._orY = pos->transform.rotation.y;					//Rotation in Y
	sumo._orZ = pos->transform.rotation.z;					//Rotation in Z
	sumo._orW = pos->transform.rotation.w;					//Rotation in W

	tf::Quaternion q(pos->transform.rotation.x,pos->transform.rotation.y,pos->transform.rotation.z,pos->transform.rotation.w);
	tf::Matrix3x3 m(q);
	m.getRPY(sumo._roll,sumo._pitch,sumo._yawRAD);				//Get the Roll, Pitch, Yaw (Radians)
	sumo._yaw = sumo._yawRAD*(180/PI);					//Convert the Yaw (Radians) into Yaw (Degrees)
	//sumo._yaw = constrainAngle(sumo._yaw);					//Set the heading of the drone
}

void setSumoOffset(void)
{
	sumo_offset_x = cos(sumo._yawRAD) + sumo._posX;				//Get the position of the target using the wand's offset in X
	sumo_offset_y = sin(sumo._yawRAD) + sumo._posY;				//Get the position of the target using the wand's offset in Y
}

double GetAngleDifference(double from, double to)
{
	double difference = to - from;
	while (difference < -180) difference += 360;
	while (difference > 180) difference -= 360;
	return difference;
}

//Get Efforts

void getEffort_pos_X(const std_msgs::Float64::ConstPtr& msg)
{
	ce_pos_X = msg->data;	
}
void getEffort_pos_Y(const std_msgs::Float64::ConstPtr& msg)
{
	ce_pos_Y = msg->data;	
}
void getEffort_alt(const std_msgs::Float64::ConstPtr& msg)
{
	ce_alt = msg->data;	
}
void getEffort_hdg(const std_msgs::Float64::ConstPtr& msg)
{
	ce_hdg = -msg->data;	
}
int main(int argc, char** argv)
{
	ros::init(argc,argv,"bebop_vicon");					//Initiates the node
	ros::NodeHandle n;							//Creates the node handler
	ros::Subscriber joy_sub,bebop_sub,sumo_sub;					//Creates the subscribers of the joystick and the vicon system
	ros::Subscriber pid_pos_X,pid_pos_Y,pid_hdg,pid_alt;			//Create subscribers of the pid calculator topics
	ros::Publisher state_pos_X,state_pos_Y,state_hdg,state_alt;		//Create publishers of the pid calculator topics
	ros::Publisher sp_pos_X,sp_pos_Y,sp_hdg,sp_alt;					//Create the publishers of the setpoint 
	ros::Publisher takeoff_bebop;						//Creates the publisher of the take-off command 
	ros::Publisher land_bebop;						//Creates the publisher of the land command
	ros::Publisher cmd_vel_pub_bebop;					//Creates the publisher of the movement command
	joy_sub = n.subscribe("/joy",1000,getJoyState);				//Initiates the Joystick Subscriber
	bebop_sub = n.subscribe("/vicon/BEBOP_1_11_2_18/BEBOP_1_11_2_18",1000,getBebopPos);	//Initiates the Vicon Subscriber to detect the Drone
	sumo_sub = n.subscribe("/vicon/SUMO_11_5_2018/SUMO_11_5_2018",1000,getSumoPos); 
	pid_pos_X = n.subscribe("/bebop_sumo/control_effort_pos_X",1000,getEffort_pos_X);
	pid_pos_Y = n.subscribe("/bebop_sumo/control_effort_pos_Y",1000,getEffort_pos_Y);
	pid_alt = n.subscribe("/bebop_sumo/control_effort_alt",1000,getEffort_alt);
	pid_hdg = n.subscribe("/bebop_sumo/control_effort_hdg",1000,getEffort_hdg);		
	cmd_vel_pub_bebop = n.advertise<geometry_msgs::Twist>("/bebop/cmd_vel",1000);	//Initiates the Command publisher into  the topics
	takeoff_bebop = n.advertise<std_msgs::Empty>("/bebop/takeoff",1000);	//Initiates the Take-Off publisher into the topics 
	land_bebop = n.advertise<std_msgs::Empty>("/bebop/land",1000);		//Initiates the Land publisher into the topics
	state_pos_X = n.advertise<std_msgs::Float64>("/bebop_sumo/state_pos_X",1000);
	state_pos_Y = n.advertise<std_msgs::Float64>("/bebop_sumo/state_pos_Y",1000);
	state_alt = n.advertise<std_msgs::Float64>("/bebop_sumo/state_alt",1000);
	state_hdg = n.advertise<std_msgs::Float64>("/bebop_sumo/state_hdg",1000);
	sp_pos_X = n.advertise<std_msgs::Float64>("/bebop_sumo/setpoint_pos_X",1000);
	sp_pos_Y = n.advertise<std_msgs::Float64>("/bebop_sumo/setpoint_pos_Y",1000);
	sp_alt = n.advertise<std_msgs::Float64>("/bebop_sumo/setpoint_alt",1000);
	sp_hdg = n.advertise<std_msgs::Float64>("/bebop_sumo/setpoint_hdg",1000);


	std_msgs::Empty msg_takeoff, msg_land;					//Message to command the drone to take-off and land 
	
	//Rotation Control
	bool rot_control = true;						//Rotation Control to use a rotated matrix

	cmd_vel_bebop.linear.x = 0;						//Set every value to 0
	cmd_vel_bebop.linear.y = 0;
	cmd_vel_bebop.linear.z = 0;

	cmd_vel_bebop.angular.x = 0;	
	cmd_vel_bebop.angular.y = 0;	
	cmd_vel_bebop.angular.z = 0;

	ros::spinOnce();							//Refresh the topics
	double hdg_target = 0;							//Initial target of the heading 
	
	ros::Duration(2).sleep();						//Time necesary to setup the publishers	

	ros::Rate r(100);							//Setting the program to 100 Hz

	printf("\n========== T A K E O F F ==========\n");
	takeoff_bebop.publish(msg_takeoff);					//Take-off
	ros::Duration(4.5).sleep();						//Wait until the drone is flying
	while(n.ok())								//Execute the program until <Ctrl + C> is pressed
	{

		if(btn_emergency)						//If the joystick's button is pressed land the drone and finish the program
		{
			ros::Duration(0.525).sleep();
			printf("\n========== L A N D [ J S ]==========\n");
			land_bebop.publish(msg_land);				//Land the drone
			break;
		}
		
		
		setSumoOffset();
		//printf("\nX: %lf Y: %lf Yaw: %lf NX: %lf NY: %lf \n ",sumo._posX,sumo._posY,sumo._yaw, sumo_offset_x, sumo_offset_y);
		///HEADING///
		
		double hdg_target = sumo._yaw;
		
		/*double eyaw = 0 - heading;
		if(eyaw > 180)
		{
			eyaw = eyaw - 360;
		}
		if(eyaw < -180)
		{
			eyaw = eyaw + 360;
		}
		
		//std::cout<<"Heading: "<< heading << " Error (to 0): " << eyaw << std::endl;
		
		st_hdg.data=eyaw;
		stp_hdg.data=hdg_target;
		ros::spinOnce();
		
		state_hdg.publish(st_hdg);
		sp_hdg.publish(stp_hdg);
		ros::Duration(0.0001).sleep();

		ros::spinOnce();
		cmd_vel_bebop.angular.z = ce_hdg;*/

		
		double diff = GetAngleDifference(heading,hdg_target+180);

		st_hdg.data=diff;
		stp_hdg.data=0;
		ros::spinOnce();
		
		state_hdg.publish(st_hdg);
		sp_hdg.publish(stp_hdg);
		ros::Duration(0.0001).sleep();

		ros::spinOnce();
		cmd_vel_bebop.angular.z = ce_hdg;
		


		//std::cout<<"Heading: "<< heading << " Control effort: " << ce_hdg << std::endl;

		///HEADING///


		ros::spinOnce();
		st_pos_X.data=bebop._posX;
		stp_pos_X.data=sumo_offset_x;
		state_pos_X.publish(st_pos_X);
		sp_pos_X.publish(stp_pos_X);
		ros::Duration(0.0001).sleep();					//A little delay to accomplish the order
		ros::spinOnce();
		bebop._cmdX = ce_pos_X;
		st_pos_Y.data=bebop._posY;
		stp_pos_Y.data=sumo_offset_y;					
		state_pos_Y.publish(st_pos_Y);
		sp_pos_Y.publish(stp_pos_Y);
		ros::spinOnce();
		bebop._cmdY = ce_pos_Y;

		bebop.rot_cmd_x = bebop._cmdX*cos(constrainAngle(heading) * 0.0174533) + bebop._cmdY*sin(constrainAngle(heading) * 0.0174533);	
		bebop.rot_cmd_y = bebop._cmdX*sin(constrainAngle(heading) * 0.0174533) - bebop._cmdY*cos(constrainAngle(heading) * 0.0174533);
					
		cmd_vel_bebop.linear.x = bebop.rot_cmd_x;		//Set the command data in X
		cmd_vel_bebop.linear.y = -bebop.rot_cmd_y;		//Set the command data in Y
		
		st_alt.data=bebop._posZ;
		stp_alt.data=1.05;
		state_alt.publish(st_alt);
		sp_alt.publish(stp_alt);
		ros::spinOnce();		
		cmd_vel_bebop.linear.z = ce_alt;			//Set the Altitude PID to 1.10 mts
		cmd_vel_pub_bebop.publish(cmd_vel_bebop);				//Set the command to move the drone
		ROS_INFO("X: %lf  Y: %lf HDG: %lf CMDX: %lf  CMDY: %lf CMDHDG: %lf",bebop._posX,bebop._posY, heading, cmd_vel_bebop.linear.x,cmd_vel_bebop.linear.y,cmd_vel_bebop.angular.z);
		ros::spinOnce();							//Refresh the topics 
		r.sleep();

	}
	ros::Duration(2).sleep();							//Wait 2 seconds to let the program finish some process
	ros::spinOnce();								//Refresh the topics
	if(bebop._posZ > 0.60)								//If the drone still in the air, land it
	{
		ros::Duration(0.525).sleep();
		printf("\n========== L A N D [ A L T ]==========\n");	
		land_bebop.publish(msg_land);						//Land
	}
	return 0;
}

