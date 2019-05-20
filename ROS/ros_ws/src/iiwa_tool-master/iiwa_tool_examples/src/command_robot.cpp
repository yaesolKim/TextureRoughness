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
//#define ros_rate 10.0
#define threshold 0.2
#define PI 3.14159265

iiwa_msgs::JointPosition current_joint_position, command_joint_position;
geometry_msgs::PoseStamped current_cartesian_position, command_cartesian_position;
std::string joint_position_topic, cartesian_position_topic, command_cartesian_position_topic, command_joint_position_topic;

bool isRobotConnected = false, robotInit = false;
bool use_cartesian_command = true;

//float ori = 180; //relative orientation between avatar and virtual wall
float ori, x, y, z, r; //receive from unity
double pos_x, pos_y, pos_z; //type casting
int rough; //type casting
double pos_x_old, pos_y_old, pos_z_old; //calculate using poses
double theta, v_y, v_z, v;

bool compliant_control = false;

void jointPositionCallback(const iiwa_msgs::JointPosition& jp);
void cartesianPositionCallback(const geometry_msgs::PoseStamped& ps);
//void avatarCallback(const geometry_msgs::PoseStamped& ps);
void error_handling(const char * message); //Socket comm
void setRobotPose(float ori, double x, double y, double z, int r, double direction_degree);
bool checkReachability(float ori, double x, double y, double z);

int main (int argc, char **argv)
{
	// Initialize ROS
	ros::init(argc, argv, "CommandRobot");
	ros::NodeHandle nh("~");

	// ROS spinner.
	ros::AsyncSpinner spinner(1);
	spinner.start();

	// comm
	int serv_sock;//server socket
	struct sockaddr_in serv_adr, clnt_adr;
	socklen_t clnt_adr_sz;//client adress size
	int str_len;//received m length
	char Connected[BUF_SIZE];//connnect message

	char message[] = "Hello, I'm Ubuntu, Server!";
	unsigned char message_rec[8];
	unsigned char VAL[8];
	// comm: create socket
	serv_sock = socket(PF_INET, SOCK_DGRAM, 0);
	if (serv_sock == -1) {
		error_handling ("UDP socket creation error");
	}
	else {
		ROS_INFO("1. success to creating UDP socket\n");
	}

	socklen_t optlen;
	int option;
	optlen=sizeof(option);
	option = 1;
	setsockopt(serv_sock, SOL_SOCKET, SO_REUSEADDR,(void*)&option, optlen);

	// comm: memset
	memset(&serv_adr, 0, sizeof(serv_adr));
	serv_adr.sin_family = AF_INET;
	serv_adr.sin_addr.s_addr = htonl(INADDR_ANY);
	serv_adr.sin_port = htons(portnum);

	// comm: binding -> assign ip address and port number
	if (bind(serv_sock, (struct sockaddr *)&serv_adr, sizeof(serv_adr)) == -1) {
		error_handling("bind() error");
	} else {
		ROS_INFO("2. success binding\n");
	}

	// comm: Receive connectM from Client
	clnt_adr_sz = sizeof(clnt_adr);//client address size
	str_len = recvfrom(serv_sock, Connected, BUF_SIZE, 0, (struct sockaddr *)&clnt_adr, &clnt_adr_sz);//receive M(Connected) from client
	Connected[str_len] = 0;

	ROS_INFO("3. message from client: %s\n", Connected);

	sendto(serv_sock, message, sizeof(message), 0, (struct sockaddr *)&clnt_adr, clnt_adr_sz); //End of the connection testing
	printf("4. send message to client\n");

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

	///// robot initialization - done after calibration in Unity
	while (ros::ok())	{
		if (isRobotConnected) {
			ROS_INFO("4. Robot connected.");

			ori = 90.0;
			pos_x = 750.0;
			pos_y = 0.0;
			pos_z = 486.33;
			rough = 0;
			theta = 0.0;

			pos_x_old = pos_x;
			pos_y_old = pos_y;
			pos_z_old = pos_z;

			command_cartesian_position = current_cartesian_position;
			setRobotPose(ori, pos_x, pos_y, pos_z, rough, theta);
			pub_cartesian_command.publish(command_cartesian_position);
			ROS_INFO("Robot moves backward.");
			ros::Duration(3.0).sleep();

			robotInit = true;
			break;
		}
		else {
			ROS_ERROR("Robot is not connected...");
			ros::Duration(1.0).sleep();
		}
	} ///// end of the robot initialization

	//start haptic loop
	while (ros::ok() && robotInit)
	{
		if (isRobotConnected && use_cartesian_command)
		{
			//waiting for message from UNITY. if there is no message, hold robot configuration and wait
			//printf("waiting for message from UNITY!\n");
			str_len = recvfrom(serv_sock, message_rec, 20, 0, (struct sockaddr *)&clnt_adr, &clnt_adr_sz);
			memcpy(&ori,	&message_rec,			sizeof(ori));
			memcpy(&x,	&message_rec[4],	sizeof(x));
			memcpy(&y,	&message_rec[8],	sizeof(y));
			memcpy(&z,	&message_rec[12],	sizeof(z));
			memcpy(&r,	&message_rec[16],	sizeof(r));
			//message_rec[str_len] = 0;

			//control mode is decided by size of the ori
			if (ori > 400) //dynamic object
			{
				compliant_control = true;
				ori = ori - 600;
			}
			else //static object
			{
				compliant_control = false;
			}

			pos_x = x;
			pos_y = y;
			pos_z = z;

			v_y = pos_y - pos_y_old;
			v_z = pos_z - pos_z_old;
			v = sqrt(pow(v_y, 2) + pow(v_z, 2));
			theta = (asin(v_z/v)*180) / PI;
			rough = int(r);

			//Check reachability roughly. TODO: add HWALL code later
			if(pos_x_old != pos_x || pos_y_old != pos_y || pos_z_old != pos_z)
			{
				setRobotPose(ori, pos_x, pos_y, pos_z, rough, theta);
				//ROS_INFO("set pos x: %f, y: %f, z: %f", command_cartesian_position.pose.position.x, command_cartesian_position.pose.position.y, command_cartesian_position.pose.position.z);
			}

			bool reachable = checkReachability(ori, pos_x, pos_y, pos_z);

			if(!compliant_control)
			{
				if(reachable)
				{
					pub_cartesian_command.publish(command_cartesian_position);
					//ROS_INFO("publish ori x: %f", ori);
				}
			}

			else //compliant control
			{
				//ROS_INFO("compliant control");
				// Setting Cartesian Impedance mode
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

				//pub_cartesian_command.publish(command_cartesian_position);

				client.call(config);
			}

			pos_x_old = pos_x;
			pos_y_old = pos_y;
			pos_z_old = pos_z;
		}
	}//end of the haptic loop


	std::cerr<<"Stopping spinner..."<<std::endl;
	spinner.stop();

	std::cerr<<"Bye!"<<std::endl;

	return 0;

};
/////////////////////////////end of the main//////////////////////////

void setRobotPose(float ori_, double x_, double y_, double z_, int r_, double direction_degree)
{
	command_cartesian_position.pose.position.x = x_;
	command_cartesian_position.pose.position.y = y_;
	command_cartesian_position.pose.position.z = z_;

	/*
	if(ori_<0) {
	ori_ = ori_ + 180;
}
*/
command_cartesian_position.pose.orientation.x = (ori_*PI)/180; //ori in radian
command_cartesian_position.pose.orientation.y = ((22.5 * floor(r_/10) - direction_degree) * PI)/180; // relative angle!!! 0,1,2,3,4
command_cartesian_position.pose.orientation.z = 1.5708; //fix this value for vertical surface
command_cartesian_position.pose.orientation.w = r_ % 10; //velocity-only this value in contact mode
}

bool checkReachability(float ori, double x, double y, double z)
{
	bool reach = false;
	if(x>-800 && x<800 && y>-800 && y<800 && z>0 && z<1200 )
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
