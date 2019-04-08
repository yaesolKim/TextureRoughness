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
#define portnum 9186

//#define ros_rate 10.0
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
float pos_x, pos_y, pos_z, rough;
float pos_x_old = 0, pos_y_old = 0, pos_z_old = 0;

float tex_angle, tex_vel;
bool compliant_control = false;

void jointPositionCallback(const iiwa_msgs::JointPosition& jp);
void cartesianPositionCallback(const geometry_msgs::PoseStamped& ps);
void error_handling(const char * message); //Socket comm
void setRobotPose(int ori, float x, float y, float z, int r);
bool checkReachability(int ori, float x, float y, float z);

int main (int argc, char **argv)
{
	// Initialize ROS
	ros::init(argc, argv, "CommandRobot");
	ros::NodeHandle nh("~");

	// ROS spinner.
	ros::AsyncSpinner spinner(1);
	spinner.start();

	//Socket comm
	int serv_sock, clnt_sock;//server socket
	struct sockaddr_in serv_adr, clnt_adr;
	socklen_t clnt_adr_sz;//client adress size
	int str_len;//received m length

	char message[] = "Hello, I'm Ubuntu, Server!";
	unsigned char message_rec[8];

	//socket creation
	serv_sock = socket (PF_INET, SOCK_STREAM, 0); //create socket
	if (serv_sock == -1) {
		error_handling ("tcp socket creation error");
	}
	else {
		printf("1. success to creating tcp socket\n");
	}

	//memset
	memset(&serv_adr, 0, sizeof(serv_adr));
	serv_adr.sin_family = AF_INET;
	serv_adr.sin_addr.s_addr = htonl(INADDR_ANY);
	serv_adr.sin_port = htons(portnum);

	//binding -> assign ip address and port number
	if (bind(serv_sock, (struct sockaddr *)&serv_adr, sizeof(serv_adr)) == -1) {
		error_handling("bind() error");
	} else {
		ROS_INFO("2. success binding\n");
	}

	if(listen(serv_sock, 5) == -1) {
		error_handling("listen() error");
	}
	else {
		printf("3. success to listening\n");
	}

	//Receive connectM from Client
	clnt_adr_sz = sizeof(clnt_adr);//client address size
	clnt_sock = accept(serv_sock, (struct sockaddr*)&clnt_adr, &clnt_adr_sz); //call 'accept' -> accept connection
	if (clnt_sock == -1) {
		error_handling("accept() error");
	}
	printf("4. waiting for message from client\n");
	str_len = recv(clnt_sock, message_rec, 1023, 0);
	message_rec[str_len] = 0;
	printf("5. message from client: %s\n", message_rec);

	printf("6. sending message to client\n");
	send(clnt_sock, message, sizeof(message), 0); //End of the connection testing

	// Create Transform listener object
	tf::TransformListener listener;

	// Dynamic parameters. Last arg is the default value. You can assign these from a launch file.
	nh.param("joint_position_topic", joint_position_topic, std::string("/iiwa/state/JointPosition"));
	nh.param("cartesian_position_topic", cartesian_position_topic, std::string("/iiwa/state/CartesianPose"));
	nh.param("command_cartesian_position_topic", command_cartesian_position_topic, std::string("/iiwa/command/CartesianPose"));
	nh.param("command_joint_position_topic", command_joint_position_topic, std::string("/iiwa/command/JointPosition"));
  	///nh.param<std::string>("ee_link", ee_link, "tool_link_ee");////
	nh.param("use_cartesian_command", use_cartesian_command, true);

	// For Impedance control
	ros::ServiceClient client = nh.serviceClient<iiwa_msgs::ConfigureSmartServo>("/iiwa/configuration/configureSmartServo");
	iiwa_msgs::ConfigureSmartServo config;

	// Subscribers and publishers
	ros::Subscriber sub_joint_position = nh.subscribe(joint_position_topic, 1, jointPositionCallback);
	ros::Subscriber sub_cartesian_position = nh.subscribe(cartesian_position_topic, 1, cartesianPositionCallback);
	ros::Publisher pub_cartesian_command = nh.advertise<geometry_msgs::PoseStamped>(command_cartesian_position_topic, 1);
	ros::Publisher pub_joint_command = nh.advertise<iiwa_msgs::JointPosition>(command_joint_position_topic, 1);
/*
	while (ros::ok())
	{
		if (isRobotConnected) {
			ROS_INFO("Robot connected.");
			break;
		}
		else {
			ROS_ERROR("Robot is not connected...");
			ros::Duration(1.0).sleep(); // 5 seconds
		}
	}
*/
	// After finishing initialization, move the robot backward.
	if(ros::ok())// && isRobotConnected)
	{
		ROS_INFO("Robot moves backward.");
		command_cartesian_position = current_cartesian_position;
		ROS_INFO("current pos_x: %f, pos_y: %f, pos_z:%f", command_cartesian_position.pose.position.x,command_cartesian_position.pose.position.y,command_cartesian_position.pose.position.z);
		command_cartesian_position.pose.position.x -= 0.15;
		pub_cartesian_command.publish(command_cartesian_position);
		ros::Duration(5.0).sleep(); // 5 seconds
	}

	//start haptic loop
	while (ros::ok())// && isRobotConnected && use_cartesian_command)
	{
		//waiting for message from UNITY. if there is no message, hold robot configuration and wait
		//printf("waiting for message from UNITY!\n");
		str_len = recv(clnt_sock, message_rec, 20, 0);
		memcpy(&ori, &message_rec, sizeof(ori));
		memcpy(&pos_x, &message_rec[4], sizeof(pos_x));
		memcpy(&pos_y, &message_rec[8], sizeof(pos_y));
		memcpy(&pos_z, &message_rec[12], sizeof(pos_z));
		memcpy(&rough, &message_rec[16], sizeof(rough));
		message_rec[str_len] = 0;

		//control mode is decided by sign of ori
		if (ori > 400)
		{
			compliant_control = true;
			ori = ori - 500;
		}
		else
		{
			compliant_control = false;
		}

		if(pos_x_old != pos_x || pos_y_old != pos_y || pos_z_old != pos_z)
		{
			setRobotPose(ori, pos_x, pos_y, pos_z, rough); ///set roughness...
		}

		pos_x_old = pos_x;
		pos_y_old = pos_y;
		pos_z_old = pos_z;

		//command_cartesian_position = current_cartesian_position;
		setRobotPose(ori, pos_x, pos_y, pos_z, rough);
		//bool reachable = checkReachability(ori, pos_x, pos_y, pos_z);

		if(!compliant_control)
		{
			//if(reachable)
			{
				pub_cartesian_command.publish(command_cartesian_position);

			}
		}

		else //compliant control
		{
			//ROS_INFO("compliant control");
			// Setting Cartesian Impedance mode
			pub_cartesian_command.publish(command_cartesian_position);
			config.request.mode.mode = iiwa_msgs::SmartServoMode::CARTESIAN_IMPEDANCE;
			config.request.mode.relative_velocity = 0.05;
			config.request.mode.cartesian_stiffness.stiffness.x = 1000;
			config.request.mode.cartesian_stiffness.stiffness.y = 1000;
			config.request.mode.cartesian_stiffness.stiffness.z = 200;
			config.request.mode.cartesian_stiffness.stiffness.a = 280;
			config.request.mode.cartesian_stiffness.stiffness.b = 280;
			config.request.mode.cartesian_stiffness.stiffness.c = 280;

			//valid range ([0.1, 1])
			config.request.mode.cartesian_damping.damping.x = 0.7;
			config.request.mode.cartesian_damping.damping.y = 0.7;
			config.request.mode.cartesian_damping.damping.z = 0.7;
			config.request.mode.cartesian_damping.damping.a = 0.7;
			config.request.mode.cartesian_damping.damping.b = 0.7;
			config.request.mode.cartesian_damping.damping.c = 0.7;

			client.call(config);
		}
	}//end of the haptic loop


	std::cerr<<"Stopping spinner..."<<std::endl;
	spinner.stop();

	std::cerr<<"Bye!"<<std::endl;

	return 0;

};
/////////////////////////////end of the main//////////////////////////

void setRobotPose(int ori, float x, float y, float z, int r)
{
	command_cartesian_position.pose.position.x = x;
	command_cartesian_position.pose.position.y = y;
	command_cartesian_position.pose.position.z = z;

	command_cartesian_position.pose.orientation.x = 22.5 * floor(r/10); // relative angle!!! 0,1,2,3,4
	command_cartesian_position.pose.orientation.y = 1.5708;
	command_cartesian_position.pose.orientation.z = (ori*PI)/180;

	command_cartesian_position.pose.orientation.w = r % 10; //TODO: relative velocity!!!

	ROS_INFO("set x: %f, y: %f, z: %f, w:%f", command_cartesian_position.pose.orientation.x, command_cartesian_position.pose.orientation.y, command_cartesian_position.pose.orientation.z, command_cartesian_position.pose.orientation.w);

}

bool checkReachability(int ori, float x, float y, float z)
{
	bool reach = false;
	if(x>-0.8 && x<0.8 && y>-0.8 && y<0.8 && z>0 && z<1.2 )
	{
			reach = true;
	}

	return reach;
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
