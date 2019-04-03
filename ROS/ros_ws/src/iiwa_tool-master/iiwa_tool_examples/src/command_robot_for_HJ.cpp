//record force/torque/position
#include "ros/ros.h"
#include "iiwa_msgs/JointPosition.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/WrenchStamped.h"
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <math.h>

iiwa_msgs::JointPosition current_joint_position, command_joint_position;
geometry_msgs::PoseStamped current_cartesian_position, command_cartesian_position;
geometry_msgs::WrenchStamped current_cartesian_wrench;

std::string joint_position_topic, cartesian_position_topic, cartesian_wrench_topic, command_cartesian_position_topic, command_joint_position_topic;
double ros_rate = 0.1;
double x, y, z; //hand position
int init;
double a, b, dist;
bool isRobotConnected = false, use_cartesian_command = true;

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

//wrench
void cartesianWrenchCallback(const geometry_msgs::WrenchStamped& cw)
{
	if (!isRobotConnected)
		isRobotConnected = !isRobotConnected;
	current_cartesian_wrench = cw;

}
int main (int argc, char **argv)
{
	// Initialize ROS
	ros::init(argc, argv, "CommandRobot");
	ros::NodeHandle nh("~");

	// ROS spinner.
	ros::AsyncSpinner spinner(1);
	spinner.start();

	// Dynamic parameters. Last arg is the default value. You can assign these from a launch file.
	nh.param("joint_position_topic", joint_position_topic, std::string("/iiwa/state/JointPosition"));
	nh.param("cartesian_position_topic", cartesian_position_topic, std::string("/iiwa/state/CartesianPose"));
	nh.param("cartesian_wrench_topic", cartesian_wrench_topic, std::string("/iiwa/state/CartesianWrench"));
	nh.param("command_cartesian_position_topic", command_cartesian_position_topic, std::string("/iiwa/command/CartesianPose"));
	nh.param("command_joint_position_topic", command_joint_position_topic, std::string("/iiwa/command/JointPosition"));
	nh.param("use_cartesian_command", use_cartesian_command, true);

	// Dynamic parameter to choose the rate at wich this node should run
	//nh.param("ros_rate", ros_rate, 0.1); // 0.1 Hz = 10 seconds
	nh.param("ros_rate", ros_rate, 2.0);
	ros::Rate* loop_rate_ = new ros::Rate(ros_rate);

	// Subscribers and publishers
	ros::Subscriber sub_joint_position = nh.subscribe(joint_position_topic, 1, jointPositionCallback);
	ros::Subscriber sub_cartesian_position = nh.subscribe(cartesian_position_topic, 1, cartesianPositionCallback);
	ros::Subscriber sub_cartesian_wrench = nh.subscribe(cartesian_wrench_topic, 1, cartesianWrenchCallback);
	//ros::Subscriber sub_hand_cartesian_position = nh.subscribe("hand_position_msg", 100, msgPositionCallback);
	ros::Publisher pub_cartesian_command = nh.advertise<geometry_msgs::PoseStamped>(command_cartesian_position_topic, 1);
	ros::Publisher pub_joint_command = nh.advertise<iiwa_msgs::JointPosition>(command_joint_position_topic, 1);

	int direction = 1;

	while (ros::ok())
	{
		if (isRobotConnected)
		{

			// Printing out the current cartesian position
      ROS_INFO("Current Joint Position is : [ %3.3f, %3.3f, %3.3f, %3.3f, %3.3f, %3.3f, %3.3f]", 
	       current_joint_position.position.a1, 
	       current_joint_position.position.a2, 
	       current_joint_position.position.a3, 
	       current_joint_position.position.a4, 
	       current_joint_position.position.a5, 
	       current_joint_position.position.a6, 
	       current_joint_position.position.a7);
/*
			ROS_INFO("Current Cartesian Position is : [ %3.3f, %3.3f, %3.3f, %3.3f, %3.3f, %3.3f, %3.3f]",
			current_cartesian_position.pose.position.x,
			current_cartesian_position.pose.position.y,
			current_cartesian_position.pose.position.z,
			current_cartesian_position.pose.orientation.x,
			current_cartesian_position.pose.orientation.y,
			current_cartesian_position.pose.orientation.z,
			current_cartesian_position.pose.orientation.w);
*/
/*
			ROS_INFO("Current Cartesian Wrench force / torque is : [ %3.3f, %3.3f, %3.3f / %3.3f, %3.3f, %3.3f ]",
			current_cartesian_wrench.wrench.force.x,
			current_cartesian_wrench.wrench.force.y,
			current_cartesian_wrench.wrench.force.z,
			current_cartesian_wrench.wrench.torque.x,
			current_cartesian_wrench.wrench.torque.y,
			current_cartesian_wrench.wrench.torque.z);
*/
			if (use_cartesian_command)
			{
				// Here we set the new commanded cartesian position, we just move the tool TCP 10 centemeters down and back up, every 20 seconds.
				//command_cartesian_position = current_cartesian_position;
/*
				//left side
				command_cartesian_position.pose.position.x = ((x*(-771)/(858)) + 825 ) /1000 + 0.1;
				command_cartesian_position.pose.position.y = -0.64;
				command_cartesian_position.pose.position.z = ((y*(-434)/(659)) + 826 ) /1000 + 0.08;

				command_cartesian_position.pose.orientation.x = 0.5;
				command_cartesian_position.pose.orientation.y = 0.5;
				command_cartesian_position.pose.orientation.z = -0.5;
				command_cartesian_position.pose.orientation.w = 0.5;

				// Command position is published and executed by the robot (if the robot can achieve that position)
				a = (current_cartesian_position.pose.position.x - command_cartesian_position.pose.position.x);
				b = (current_cartesian_position.pose.position.z - command_cartesian_position.pose.position.z);
				dist = sqrt(a*a + b*b);

				if(dist > 0.2)
				{
					ROS_INFO("FAR, command - X: %f, Z: %f", command_cartesian_position.pose.position.x, command_cartesian_position.pose.position.z);
					command_cartesian_position.pose.position.x = ( (command_cartesian_position.pose.position.x - current_cartesian_position.pose.position.x) / dist ) * 0.2 + current_cartesian_position.pose.position.x;
					command_cartesian_position.pose.position.z = ( (command_cartesian_position.pose.position.z - current_cartesian_position.pose.position.z) / dist ) * 0.2 + current_cartesian_position.pose.position.z;
				}

				ROS_INFO("k: %f, %f, %f", x, y, z);
				ROS_INFO("current - X: %f, Z: %f", current_cartesian_position.pose.position.x, current_cartesian_position.pose.position.z);
				ROS_INFO("command - X: %f, Z: %f", command_cartesian_position.pose.position.x, command_cartesian_position.pose.position.z);
*/
				//pub_cartesian_command.publish(command_cartesian_position);
			}

			else {
				ROS_INFO("joint");
				//command_joint_position = current_joint_position;
				//command_joint_position.position.a4 -= direction * 0.0872665; // Adding/Subtracting 5 degrees (in radians) to the 4th joint

				//pub_joint_command.publish(command_joint_position); // Command position is published and executed by the robot (if the joint limit is not exceeded)

			}
			//loop_rate_->sleep(); // Sleep for some millisecond. The while loop will run every 10 seconds in this example.
		}
		else {
			ROS_ERROR("Robot is not connected...");
			ros::Duration(1.0).sleep(); // 5 seconds
		}
	}

	std::cerr<<"Stopping spinner..."<<std::endl;
	spinner.stop();

	std::cerr<<"Bye!"<<std::endl;

	return 0;

};
