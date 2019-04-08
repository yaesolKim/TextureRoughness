#include "ros/ros.h"
#include "iiwa_msgs/JointPosition.h"
#include "geometry_msgs/PoseStamped.h"

iiwa_msgs::JointPosition current_joint_position;
std::string joint_position_topic;
double ros_rate = 0.1;
bool isRobotConnected = false;

void jointPositionCallback(const iiwa_msgs::JointPosition& jp)
{
  if (!isRobotConnected)
    isRobotConnected = !isRobotConnected;
  current_joint_position = jp;
}

int main (int argc, char **argv) {
  
  // Initialize ROS
  ros::init(argc, argv, "ReadRobotState");
  ros::NodeHandle nh("~");
  
  // ROS spinner.
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  // Dynamic parameter to choose the topic to use. Last arg is the default value.
  nh.param("joint_position_topic", joint_position_topic, std::string("/iiwa/state/JointPosition"));
  // Dynamic parameter to choose the rate at wich this node should run
  nh.param("ros_rate", ros_rate, 0.1); // 0.1 Hz = 10 seconds
  ros::Rate* loop_rate_ = new ros::Rate(ros_rate);
  
  // Subscriber to the given topic
  ros::Subscriber sub = nh.subscribe(joint_position_topic, 1, jointPositionCallback);
  ROS_INFO("START");
  while (ros::ok()) {
    if (isRobotConnected) {
      ROS_INFO("Robot is connected...");
      ROS_INFO("Current Joint Position is : [ %3.3f, %3.3f, %3.3f, %3.3f, %3.3f, %3.3f, %3.3f]", 
	       current_joint_position.position.a1, 
	       current_joint_position.position.a2, 
	       current_joint_position.position.a3, 
	       current_joint_position.position.a4, 
	       current_joint_position.position.a5, 
	       current_joint_position.position.a6, 
	       current_joint_position.position.a7);
      
      // Sleep for some milliseconds. The while loop will run every 10 seconds in this example.
      loop_rate_->sleep();
    }
    else {
      ROS_ERROR("Robot is not connected...");
      ros::Duration(5.0).sleep(); // 5 seconds
    }
  }
  
  std::cerr<<"Stopping spinner..."<<std::endl;
  spinner.stop();
  
  std::cerr<<"Bye!"<<std::endl;
  
  return 0;
  
}; 
