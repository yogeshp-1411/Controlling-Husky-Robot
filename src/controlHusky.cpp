#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "math.h"
#include "tf/tf.h"
#include "angles/angles.h"

float current_x, current_y;
double theta_degree;
float goal_x = 5;
float goal_y = 1;

void callBack(const nav_msgs::Odometry::ConstPtr& msg)
{
	current_x = msg->pose.pose.position.x;
	current_y = msg->pose.pose.position.y;

	tf::Quaternion q( msg->pose.pose.orientation.x,
		 	msg->pose.pose.orientation.y,
			msg->pose.pose.orientation.z,
			msg->pose.pose.orientation.w);

	double theta_yaw = tf::getYaw(q);
	theta_degree = angles::to_degrees(angles::normalize_angle_positive(theta_yaw));
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "controlHusky");
	ros::NodeHandle n;
	
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/twist_marker_server/cmd_vel", 1000);
	ros::Subscriber husky_sub = n.subscribe("/odometry/filtered", 1000, callBack);

	ros::Rate loop_rate(10);
	
	double goal_angle;

	while (ros::ok())
	{
		geometry_msgs::Twist motion;

		printf("Diff_x: %f 	Diff_y: %f\n", round(goal_x - current_x), round(goal_y - current_y));
		
		if(round(goal_x - current_x) ==0 && round(goal_y - current_y) ==0)
		{//once reached the destination, align the robot in x direction
			if(round(theta_degree) != 0.00)
			{
				motion.angular.z = 0.1;
				pub.publish(motion);
			}
			else
			{
				ros::shutdown();
			}

		}
		else if((round(goal_x - current_x) !=0) || (round(goal_y - current_y)!=0))
		{
			double goal = (atan(goal_y/goal_x));
			goal_angle =  angles::to_degrees(angles::normalize_angle_positive(goal));
			//goal_angle =  angles::to_degrees(angles::normalize_angle(goal));
			printf("Goal Angle: %f Current Angle: %f \n", goal_angle, (theta_degree));

			double diff = abs(theta_degree - goal_angle);
			if (diff != 0)
			{
			//align the robot
		
				motion.linear.x = 0.0;
				motion.linear.y = 0.0;
				motion.linear.z = 0.0;
				motion.angular.x = 0.0;
				motion.angular.y = 0.0;		
				motion.angular.z = 0.1;
		
				pub.publish(motion);
			}
			else
			{//go straight
				motion.linear.x = 0.2;
				motion.linear.y = 0.0;
				motion.linear.z = 0.0;
				motion.angular.x = 0.0;
				motion.angular.y = 0.0;		
				motion.angular.z = 0.0;
		
				pub.publish(motion);		
			}
		}
		
		ros::spinOnce();
		loop_rate.sleep();
	}	
	return 0;
}

