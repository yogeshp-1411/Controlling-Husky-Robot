//Code to subscribe to /odometry/filtered and retrieve the needed information.
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"

void callBack(const nav_msgs::Odometry::ConstPtr& msg)
{
	ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
	ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "subscriber_odom");
	ros::NodeHandle n;
	
	ros::Subscriber husky_sub = n.subscribe("/odometry/filtered", 1000, callBack);

	ros::spin();
}

