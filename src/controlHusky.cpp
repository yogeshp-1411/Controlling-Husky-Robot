#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "math.h"
#include "tf/tf.h"

void callBack(const nav_msgs::Odometry::ConstPtr& msg)
{
	float x = msg->pose.pose.position.x;
	float y = msg->pose.pose.position.y;
	
	//ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
	//ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	//ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);

	/*float orientation[3];
	orientation[0] = msg->pose.pose.orientation.x;
	orientation[1] = msg->pose.pose.orientation.y;
	orientation[2] = msg->pose.pose.orientation.z;
	orientation[3] = msg->pose.pose.orientation.w;*/
	//printf("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f] \n", orientation[0], orientation[1], orientation[2], orientation[3]);

	tf::Quaternion q( msg->pose.pose.orientation.x,
		 	msg->pose.pose.orientation.y,
			msg->pose.pose.orientation.z,
			msg->pose.pose.orientation.w);

	//orientation.getRPY(roll, pitch, yaw);

	double theta = tf::getYaw(q);
	printf("%f \n", theta);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "controlHusky");
	ros::NodeHandle n;
	
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/twist_marker_server/cmd_vel", 1000);
	ros::Subscriber husky_sub = n.subscribe("/odometry/filtered", 1000, callBack);

	ros::Rate loop_rate(10);
	
	while (ros::ok())
	{
		geometry_msgs::Twist motion;
		
		motion.linear.x = 0.2;
		motion.linear.y = 0.0;
		motion.linear.z = 0.0;
		motion.angular.x = 0.0;
		motion.angular.y = 0.0;		
		motion.angular.z = 0.1;
		
		pub.publish(motion);

		ros::spinOnce();
		loop_rate.sleep();
	}	
	return 0;
}

