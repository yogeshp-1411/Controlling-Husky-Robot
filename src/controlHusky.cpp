#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "math.h"
#include "tf/tf.h"
#include "angles/angles.h"

float current_x, current_y;
double theta_degree;
float goal_x = 5;
float goal_y = 3;

void callBack(const nav_msgs::Odometry::ConstPtr& msg)
{
	current_x = msg->pose.pose.position.x;
	current_y = msg->pose.pose.position.y;
	
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

	double theta_yaw = tf::getYaw(q);
	theta_degree = angles::to_degrees(angles::normalize_angle_positive(theta_yaw));
//	theta_degree += 90; 
	//printf("%f %f\n", theta_yaw, theta_angle);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "controlHusky");
	ros::NodeHandle n;
	
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/twist_marker_server/cmd_vel", 1000);
	ros::Subscriber husky_sub = n.subscribe("/odometry/filtered", 1000, callBack);

	ros::Rate loop_rate(10);
	
	double goal_angle;
	//double goal_angle_1, goal_angle_2;
	while (ros::ok())
	{
		printf("Diff_x: %f 	Diff_y: %f\n", (goal_x - current_x), (goal_y - current_y));

		//while( (((goal_x - current_x) > 0.3) || ((goal_x - current_x) < -0.3)) && (((goal_y - current_y) >0.3) || ((goal_y - current_y) < -0.3)) )
		//while((goal_x - current_x)>0 && (goal_y - current_y)>0)
		if((goal_x - current_x)>0 && (goal_y - current_y)>0)
		{
			double goal = (atan(goal_y/goal_x));
			goal_angle =  angles::to_degrees(angles::normalize_angle_positive(goal));
			//printf("Goal Angle: %f Current Angle: %f \n", goal_angle, (theta_degree));
		
			geometry_msgs::Twist motion;

			double diff = abs(theta_degree - goal_angle);
	//		printf("%f\n", diff);

			if (diff != 0)
			{
	//		printf("%f \n", theta_degree);
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

