//for testing orientations
//Lets add door using impedance control
#include "ros/ros.h"
#include "iiwa_msgs/JointPosition.h"
#include "geometry_msgs/PoseStamped.h"
#include "iiwa_msgs/ConfigureSmartServo.h" //for impedance control
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <math.h>

//Socket comm
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

// read reachable plane files
#include <iostream>
#include <fstream>

using namespace std;

//Socket comm
#define BUF_SIZE 28
#define portnum 9120

//#define ros_rate 100.0
#define threshold 0.2
#define PI 3.14159265

//nh.param("ros_rate", ros_rate, 0.1); // 0.1 Hz = 10 seconds
//ros::Rate* loop_rate_ = new ros::Rate(ros_rate);

iiwa_msgs::JointPosition current_joint_position, command_joint_position;
geometry_msgs::PoseStamped current_cartesian_position, command_cartesian_position;
std::string joint_position_topic, cartesian_position_topic, command_cartesian_position_topic, command_joint_position_topic;

bool isRobotConnected = false, robotInit = false;
bool use_cartesian_command = true;

float ori = 180; //relative orientation between avatar and virtual wall
float pos_x, pos_y, pos_z;
int cluster, roughness;
float tex_angle, tex_vel;
bool compliant_control = false;

void jointPositionCallback(const iiwa_msgs::JointPosition& jp);
void cartesianPositionCallback(const geometry_msgs::PoseStamped& ps);
void error_handling(const char * message); //Socket comm
void setRobotPose(int ori, float x, float y, float z);

float xxx = 0.0;
int main (int argc, char **argv)
{
	// Initialize ROS
	ros::init(argc, argv, "CommandRobot");
	ros::NodeHandle nh("~");

	// ROS spinner.
	ros::AsyncSpinner spinner(1);
	spinner.start();

// Create Transform listener object
tf::TransformListener listener;

// Dynamic parameters. Last arg is the default value. You can assign these from a launch file.
nh.param("joint_position_topic", joint_position_topic, std::string("/iiwa/state/JointPosition"));
nh.param("cartesian_position_topic", cartesian_position_topic, std::string("/iiwa/state/CartesianPose"));
nh.param("command_cartesian_position_topic", command_cartesian_position_topic, std::string("/iiwa/command/CartesianPose"));
nh.param("command_joint_position_topic", command_joint_position_topic, std::string("/iiwa/command/JointPosition"));
nh.param("use_cartesian_command", use_cartesian_command, true);

// For Impedance control
ros::ServiceClient client = nh.serviceClient<iiwa_msgs::ConfigureSmartServo>("/iiwa/configuration/configureSmartServo");
iiwa_msgs::ConfigureSmartServo config;

// Subscribers and publishers
ros::Subscriber sub_joint_position = nh.subscribe(joint_position_topic, 1, jointPositionCallback);
ros::Subscriber sub_cartesian_position = nh.subscribe(cartesian_position_topic, 1, cartesianPositionCallback);
ros::Publisher pub_cartesian_command = nh.advertise<geometry_msgs::PoseStamped>(command_cartesian_position_topic, 1);
ros::Publisher pub_joint_command = nh.advertise<iiwa_msgs::JointPosition>(command_joint_position_topic, 1);

while (ros::ok())
{
	if (isRobotConnected) {
		ROS_INFO("current pos x, y, z: %f, %f, %f", current_cartesian_position.pose.position.x, current_cartesian_position.pose.position.y, current_cartesian_position.pose.position.z);
		break;
	}
	else {
		ROS_ERROR("Robot is not connected...");
		ros::Duration(1.0).sleep(); // 5 seconds
	}
}
// After finishing initialization, move the robot backward.
if(ros::ok() && isRobotConnected)
{

	command_cartesian_position = current_cartesian_position;
	ROS_INFO("Robot moves goal pose");
	xxx = 800.0;
	setRobotPose(180, xxx, 0.0, 500.0);
	ROS_INFO("current pos x, y, z: %f, %f, %f", command_cartesian_position.pose.position.x, command_cartesian_position.pose.position.y, command_cartesian_position.pose.position.z);

}

//start haptic loop
	while (ros::ok() && isRobotConnected) {
		//command_cartesian_position = current_cartesian_position;
		xxx += 0.0003;
		//setRobotPose(180, xxx, 0.0, 500.0);
		setRobotPose(195, xxx, 0.0, 500.0);
		//setRobotPose(165, xxx, 0.0, 500.0);
		//setRobotPose(210, xxx, 0.0, 500.0);
		//setRobotPose(150, xxx, 0.0, 500.0);
		pub_cartesian_command.publish(command_cartesian_position);
		ROS_INFO("xxx:  %f", xxx);
		//ros::Duration(0.1).sleep();

}//end of the haptic loop


std::cerr<<"Stopping spinner..."<<std::endl;
spinner.stop();

std::cerr<<"Bye!"<<std::endl;

return 0;

};
/////////////////////////////end of the main//////////////////////////


void setRobotPose(int ori, float x, float y, float z)
{
	if(ori==225) //ori_1
	{
		//ROS_INFO("ori_1");
		command_cartesian_position.pose.orientation.z = -0.785398;
	}

	else if(ori==210) //ori_2
	{
		//ROS_INFO("ori_2");
		command_cartesian_position.pose.orientation.z = -0.523599;
	}

	else if(ori==195) //ori_3
	{
		//ROS_INFO("ori_3");
		command_cartesian_position.pose.orientation.z = -0.261799;
	}

	else if(ori==180) //ori_4
	{
		//ROS_INFO("ori_4");
		command_cartesian_position.pose.orientation.z = 0;
	}

	else if(ori==165) //ori_5
	{
		//ROS_INFO("ori_5");
		command_cartesian_position.pose.orientation.z = 0.261799;
	}
	else if(ori==150) //ori_6
	{
		//ROS_INFO("ori_6");
		command_cartesian_position.pose.orientation.z = 0.523599;
	}

	else if(ori==135) //ori_7
	{
		//ROS_INFO("ori_7");
		command_cartesian_position.pose.orientation.z = 0.785398;
	}

	command_cartesian_position.pose.position.x = x;
	command_cartesian_position.pose.position.y = y;
	command_cartesian_position.pose.position.z = z;

	command_cartesian_position.pose.orientation.x = 0;
	command_cartesian_position.pose.orientation.y = 1.5708;
	command_cartesian_position.pose.orientation.w = 0; //garbage
}



void jointPositionCallback(const iiwa_msgs::JointPosition& jp)
{
	if (!isRobotConnected)
	isRobotConnected = !isRobotConnected;
	current_joint_position = jp;
}

void cartesianPositionCallback(const geometry_msgs::PoseStamped& ps)
{
	if (!isRobotConnected)
	isRobotConnected = !isRobotConnected;
	current_cartesian_position = ps;
}

//Socket comm
void error_handling(const char * message)
{
	fputs(message, stderr);
	fputc('\n', stderr);
	exit(1);
}
