#include "ros/ros.h"
#include "iiwa_msgs/JointPosition.h"
#include "geometry_msgs/PoseStamped.h"
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

// read reachable planefiles
#include <iostream>
#include <fstream>

//Socket comm
#define BUF_SIZE 28
#define portnum 9184

//#define ros_rate 10.0
#define threshold 0.2

using namespace std;

iiwa_msgs::JointPosition current_joint_position, command_joint_position;
geometry_msgs::PoseStamped current_cartesian_position, command_cartesian_position;
std::string joint_position_topic, cartesian_position_topic, command_cartesian_position_topic, command_joint_position_topic;

bool isRobotConnected = false, use_cartesian_command = true;
bool robotInit = false;

float x, y, z; //hand position relative to head position from openni_tracker node
float forward_headx, turn_heady, headz; //avatar vel/rot from openni_tracker node

float temp1=0, temp2=0, temp3=0, temp4=0, temp5=0;
int init;
double a, b, dist;
bool movemode = true;
float ori_f = 4;
float pos_rel_wall = 0;

// PPRM
float ori_1[61][51][2], ori_2[61][51][2], ori_3[61][51][2], ori_4[61][51][2], ori_5[61][51][2], ori_6[61][51][2], ori_7[61][51][2];

bool data_not_same;

void jointPositionCallback(const iiwa_msgs::JointPosition& jp);
void cartesianPositionCallback(const geometry_msgs::PoseStamped& ps);
void avatarCallback(const geometry_msgs::PoseStamped& ps);
void error_handling(char * message); //Socket comm
void readPPRM(int pn, int zn); // read PPRM
bool inside_PPRM(int ori, float commandx, float commandy, float commandz);

int main (int argc, char **argv)
{
	readPPRM(61, 51);
	//ROS_INFO("ori_4[3][21][0] = %.3f, ori_4[3][21][1] = %.3f", ori_4[3][21][0], ori_4[3][21][1]);

	// Initialize ROS
	ros::init(argc, argv, "CommandRobot");
	ros::NodeHandle nh("~");

	// ROS spinner.
	ros::AsyncSpinner spinner(1);
	spinner.start();

	//Socket comm
	int serv_sock;//server socket
	socklen_t clnt_adr_sz;//client adress size
	struct sockaddr_in serv_adr;
	struct sockaddr_in clnt_adr;
	int str_len;//received m length
	char Connected[BUF_SIZE];//connnect message
	float avatar_vel[5]={0.0}; // save data about avatar(hand x, hand y, hand z, forward, turn)
	unsigned char floatTobyte[BUF_SIZE];
	unsigned char VAL[8];
	uint8_t * pointer = (uint8_t *)avatar_vel;

	//socket creation
	serv_sock = socket(PF_INET, SOCK_DGRAM, 0);
	if (serv_sock == -1) {
		error_handling("UDP socket creation error");
	}

	//memset
	memset(&serv_adr, 0, sizeof(serv_adr));
	serv_adr.sin_family = AF_INET;
	serv_adr.sin_addr.s_addr = htonl(INADDR_ANY);
	serv_adr.sin_port = htons(portnum);
	ROS_INFO("1.memset");

	//binding
	if (bind(serv_sock, (struct sockaddr *)&serv_adr, sizeof(serv_adr)) == -1) {
		error_handling("bind() error");
	} else {
		ROS_INFO("2.success binding");
	}

	//Receive connectM from Client
	clnt_adr_sz = sizeof(clnt_adr);//client address size
	str_len = recvfrom(serv_sock, Connected, BUF_SIZE, 0, (struct sockaddr *)&clnt_adr, &clnt_adr_sz);//receive M(Connected) from client
	Connected[str_len]=0;
	ROS_INFO("3.Received from Client: %s\n", Connected);

	// Create Transform listener object
	tf::TransformListener listener;

	// Dynamic parameters. Last arg is the default value. You can assign these from a launch file.
	nh.param("joint_position_topic", joint_position_topic, std::string("/iiwa/state/JointPosition"));
	nh.param("cartesian_position_topic", cartesian_position_topic, std::string("/iiwa/state/CartesianPose"));
	nh.param("command_cartesian_position_topic", command_cartesian_position_topic, std::string("/iiwa/command/CartesianPose"));
	nh.param("command_joint_position_topic", command_joint_position_topic, std::string("/iiwa/command/JointPosition"));
	nh.param("use_cartesian_command", use_cartesian_command, true);

	// Dynamic parameter to choose the rate at wich this node should run
	//nh.param("ros_rate", ros_rate, 1.0); // 0.5 Hz = 10 seconds
	//ros::Rate* loop_rate_ = new ros::Rate(ros_rate);

	// Subscribers and publishers
	ros::Subscriber avatar_sub = nh.subscribe("avatar_msg", 100, avatarCallback);
	ros::Subscriber sub_joint_position = nh.subscribe(joint_position_topic, 1, jointPositionCallback);
	ros::Subscriber sub_cartesian_position = nh.subscribe(cartesian_position_topic, 1, cartesianPositionCallback);
	ros::Publisher pub_cartesian_command = nh.advertise<geometry_msgs::PoseStamped>(command_cartesian_position_topic, 1);
	ros::Publisher pub_joint_command = nh.advertise<iiwa_msgs::JointPosition>(command_joint_position_topic, 1);

	// After finishing initialization, move the robot backward.
	while(ros::ok() && !robotInit)
	{
		if(x==1)
		{
			ROS_INFO("Robot moves backward.");
			command_cartesian_position = current_cartesian_position;
			command_cartesian_position.pose.position.x -= 0.15;
			pub_cartesian_command.publish(command_cartesian_position);

			robotInit = true;
		}
	}

	//start haptic loop
	while (ros::ok() && robotInit)
	{
		if (isRobotConnected && use_cartesian_command)
		{
			//for robot: send hand position with respect to robot base
			command_cartesian_position = current_cartesian_position;

			//ori_4
			command_cartesian_position.pose.orientation.x = 0;
			command_cartesian_position.pose.orientation.y = 0.707;
			command_cartesian_position.pose.orientation.z = 0;
			command_cartesian_position.pose.orientation.w = 0.707;

			//ori_1
			//command_cartesian_position.pose.orientation.x = -0.2706;
			//command_cartesian_position.pose.orientation.y = 0.6533;
			//command_cartesian_position.pose.orientation.z = 0.2706;
			//command_cartesian_position.pose.orientation.w = 0.6533;

			//ori_2
			//command_cartesian_position.pose.orientation.x = -0.183;
			//command_cartesian_position.pose.orientation.y = 0.683;
			//command_cartesian_position.pose.orientation.z = 0.183;
			//command_cartesian_position.pose.orientation.w = 0.683;

			//ori_3
			//command_cartesian_position.pose.orientation.x = -0.0923;
			//command_cartesian_position.pose.orientation.y = 0.7011;
			//command_cartesian_position.pose.orientation.z = 0.0923;
			//command_cartesian_position.pose.orientation.w = 0.7011;

			//ori_5
			//command_cartesian_position.pose.orientation.x = 0.0923;
			//command_cartesian_position.pose.orientation.y = 0.7011;
			//command_cartesian_position.pose.orientation.z = -0.0923;
			//command_cartesian_position.pose.orientation.w = 0.7011;

			//ori_6
			//command_cartesian_position.pose.orientation.x = 0.183;
			//command_cartesian_position.pose.orientation.y = 0.683;
			//command_cartesian_position.pose.orientation.z = -0.183;
			//command_cartesian_position.pose.orientation.w = 0.683;

			//ori_7
			//command_cartesian_position.pose.orientation.x = 0.2706;
			//command_cartesian_position.pose.orientation.y = 0.6533;
			//command_cartesian_position.pose.orientation.z = -0.2706;
			//command_cartesian_position.pose.orientation.w = 0.6533;

			//ignore the same data
			data_not_same = (temp1!=x || temp2!=y || temp3!=z || temp4!=forward_headx || temp5!=turn_heady);
			//ROS_INFO("%f, %f, %f", x, y, z);
			//if (data_not_same && inside_PPRM(4, 522.4, y, z+130))
			if (data_not_same)
			{
				//ori_4
				command_cartesian_position.pose.position.x = 0.5224;
				command_cartesian_position.pose.position.y = y/1000;
				command_cartesian_position.pose.position.z = (z + 130)/1000;

				//ori_7
				//command_cartesian_position.pose.position.x = 0.53 + (y+80)/1000;
				//command_cartesian_position.pose.position.y = (y+80)/1000;
				//command_cartesian_position.pose.position.z = (z + 130)/1000;

				temp1 = x;
				temp2 = y;
				temp3 = z;
				temp4 = forward_headx;
				temp5 = turn_heady;

				//for avatar: send hand position with respect to head position
				avatar_vel[0] = x; //hand x
				if(x!=0)
					avatar_vel[0] = (x - forward_headx)/1000;

				avatar_vel[1] = y; //hand y
				if(y!=0)
					avatar_vel[1] = (y - turn_heady)/1000;

				avatar_vel[2] = z; //hand z
				if(z!=0)
					avatar_vel[2] = (z - headz)/1000;

				avatar_vel[3] = forward_headx; //forward
				avatar_vel[4] = turn_heady; //turn


				// publish command to IIWA
				if(x!=0 && y!=0 && z!=0)
				{
					//ROS_INFO("TOUCHMODE");
					pub_cartesian_command.publish(command_cartesian_position);
				}

				//Socekt comm: transform data type from float to byte (jointArray->floatTobyte)
				for (int i=0; i<sizeof(avatar_vel); i++)
				{
					floatTobyte[i] = pointer[i];
				}

				//Send avatar intormation array(20byte) to Client (Window-Unity)
				if (sendto(serv_sock, floatTobyte, 20, 0, (struct sockaddr *)&clnt_adr, clnt_adr_sz) == -1)
				{
					error_handling("sendto() error");
				}

			}


		}
	}


	std::cerr<<"Stopping spinner..."<<std::endl;
	spinner.stop();

	std::cerr<<"Bye!"<<std::endl;

	return 0;

};
/////////////////////////////end of the main//////////////////////////




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

void avatarCallback(const geometry_msgs::PoseStamped& ps)
{
	x = ps.pose.position.x;
	y = ps.pose.position.y;
	z = ps.pose.position.z;

	forward_headx = ps.pose.orientation.x;
	turn_heady = ps.pose.orientation.y;
	headz = ps.pose.orientation.z;
}

//Socket comm
void error_handling(char * message)
{
	fputs(message, stderr);
	fputc('\n', stderr);
	exit(1);
}

void readPPRM(int pn, int zn) //number of plane(61), number of z(51)
{
	//Read PPRM
	/*	std::ifstream ori_1_file;
	ori_1_file.open("/home/glab/ros_ws/src/iiwa_tool-master/iiwa_tool_examples/reachablePlane/ori_1.txt");
	std::ifstream ori_2_file;
	ori_2_file.open("/home/glab/ros_ws/src/iiwa_tool-master/iiwa_tool_examples/reachablePlane/ori_2.txt");
	std::ifstream ori_3_file;
	ori_3_file.open("/home/glab/ros_ws/src/iiwa_tool-master/iiwa_tool_examples/reachablePlane/ori_3.txt");
	*/
	std::ifstream ori_4_file;
	ori_4_file.open("/home/glab/ros_ws/src/iiwa_tool-master/iiwa_tool_examples/PPRM/ori_4.txt");
	/*
	std::ifstream ori_5_file;
	ori_5_file.open("/home/glab/ros_ws/src/iiwa_tool-master/iiwa_tool_examples/reachablePlane/ori_5.txt");
	std::ifstream ori_6_file;
	ori_6_file.open("/home/glab/ros_ws/src/iiwa_tool-master/iiwa_tool_examples/reachablePlane/ori_6.txt");
	std::ifstream ori_7_file;
	ori_7_file.open("/home/glab/ros_ws/src/iiwa_tool-master/iiwa_tool_examples/reachablePlane/ori_7.txt");


	if (ori_1_file.is_open())
	{
	for (int j = 0; j < pn; j++)
	{
	for (int i = 0; i < zn; i++)
	{
	ori_1_file >> ori_1[j][i][0];//min
	ori_1_file >> ori_1[j][i][1];//max
}
}
}
ori_1_file.close();

if (ori_2_file.is_open())
{
for (int j = 0; j < pn; j++)
{
for (int i = 0; i < zn; i++)
{
ori_2_file >> ori_1[j][i][0];
ori_2_file >> ori_1[j][i][1];
}
}
}
ori_2_file.close();

if (ori_3_file.is_open())
{
for (int j = 0; j < pn; j++)
{
for (int i = 0; i < zn; i++)
{
ori_3_file >> ori_1[j][i][0];
ori_3_file >> ori_1[j][i][1];
}
}
}
ori_3_file.close();
*/
if (ori_4_file.is_open())
{
	for (int j = 0; j < pn; j++)
	{
		for (int i = 0; i < zn; i++)
		{
			ori_4_file >> ori_4[j][i][0];
		}
		for (int i = 0; i < zn; i++)
		{
			ori_4_file >> ori_4[j][i][1];
		}
	}
}
ori_4_file.close();
/*
if (ori_5_file.is_open())
{
for (int j = 0; j < pn; j++)
{
for (int i = 0; i < zn; i++)
{
ori_5_file >> ori_5[j][i][0];
ori_5_file >> ori_5[j][i][1];
}
}
}
ori_5_file.close();

if (ori_6_file.is_open())
{
for (int j = 0; j < pn; j++)
{
for (int i = 0; i < zn; i++)
{
ori_6_file >> ori_6[j][i][0];
ori_6_file >> ori_6[j][i][1];
}
}
}
ori_6_file.close();

if (ori_7_file.is_open())
{
for (int j = 0; j < pn; j++)
{
for (int i = 0; i < zn; i++)
{
ori_7_file >> ori_7[j][i][0];
ori_7_file >> ori_7[j][i][1];
}
}
}
ori_7_file.close();
*/
}

bool inside_PPRM(int ori, float commandx, float commandy, float commandz)
{
	int i1, i2;

	if (ori==4)
	{
		i1 = (int)((900 - commandx)/30);
		i2 = (int)((240 + commandz)/30);
		if( (ori_4[i1][i2][0] < commandy) && (ori_4[i1][i2][1] > commandy) )
		return true;
		else
		return false;
	}

	else
	return false;
}
