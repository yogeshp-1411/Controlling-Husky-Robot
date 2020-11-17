/* 
"huskyControl" Node

Description:
	This code is to demonstrate that the Husky robot reaches to user decided co-ordinates given whilst executing.
	The current_x, current_y and theta_degree values are constantly updated by subscribing to /odometry/filtered topic.
	To make the robot move, the values are published to /twist_marker_server/cmd_vel topic. 
	
Algorithm: 
	Find out the angles to the destination. As we know the destination coordinates and the current coordinates of the robot, it is straight forward to calculate the angle to the destination. This value is stored in "goal_angle". However, there are 4 cases to which the robot takes different actions. 
Case 1: current values of x and y are smaller than the destination coordinate values.
	Angle to destination = inverse tan ((goal_y-current_y)/(goal_x-current_x))
Case 2: Current value of x is larger than that of the destination coordinates x value. Current y value is smaller than destination coordinates y value.
	Angle to destination = (inverse tan ((goal_x-current_x)/(goal_y-current_y))) + 90 degrees
Case 3: Current value of x and y are larger than that of destination coordinates.
	Angle to destination = (inverse tan ((goal_y-current_y)/(goal_x-current_x))) + 180 degrees 
Case 4: Current x value is smaller than that of destination coordinates x value. Current y value is larger than destination coordinates y value. 
	Angle to destination = 360 degrees - (inverse tan ((goal_y-current_y)/(goal_x-current_x))) 
	
	The robot changes its direction of approach based on the 4 cases to result in perfect motion required to reach to the destination.
*/

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "math.h"
#include "tf/tf.h"
#include "angles/angles.h"

// The initial current_x and current_y position are set as (1,1). Otherwise, the robot is asked to go to (0,0) would not execute because the subscription to /odometry/filtered takes time to update the current positions. This results in making the robot think that it currently at (0,0) and would not take any further steps and eventually terminates.
float current_x = 1; 
float current_y = 1;
double theta_degree;
bool clock_wise = false;
float goal_x = 0;
float goal_y = 0;

void callBack(const nav_msgs::Odometry::ConstPtr& msg)
{
	current_x = (msg->pose.pose.position.x);
	current_y = (msg->pose.pose.position.y);

	tf::Quaternion q( msg->pose.pose.orientation.x,
		 	msg->pose.pose.orientation.y,
			msg->pose.pose.orientation.z,
			msg->pose.pose.orientation.w);

	double theta_yaw = tf::getYaw(q);
	theta_degree = round(angles::to_degrees(angles::normalize_angle_positive(theta_yaw)));
}

// decideTurnDirectionfunction decides the turn direction of the robot. 
// true:-> turn clockwise, 
// false :-> turn anticlockwise
void decideTurnDirection(double goal_angle, double current_angle)
{
	if((goal_angle - current_angle ) > -180 && (goal_angle - current_angle ) <= 0)
		clock_wise = true;
	else
		clock_wise = false;
			
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "controlHusky");
	ros::NodeHandle n("~");

	n.getParam("x", goal_x);
	n.getParam("y", goal_y);
	
	ros::Subscriber husky_sub = n.subscribe("/odometry/filtered", 1000, callBack);
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/twist_marker_server/cmd_vel", 1000);

	ros::Rate loop_rate(10);
	
	double goal_angle, goal, diff;
	
	while (ros::ok())
	{
		geometry_msgs::Twist motion;

		printf("Diff_x: %f 	Diff_y: %f\n", round(goal_x - current_x), round(goal_y - current_y));
		printf("Current_x: %f	Current_y: %f\n", current_x, current_y);

		if ( (round(goal_x - current_x) == 0) && (round(goal_y - current_y) == 0) )
		{//once reached the destination, align the robot in x direction
			printf("STOPPING!!!  Current Angle %f\n", theta_degree);
			if(theta_degree != 0.00)
			{
				if(theta_degree <= 180 && theta_degree > 0 )
					motion.angular.z = -0.1;
				else
					motion.angular.z = 0.1;
				pub.publish(motion);
			}
			else
			{
				ros::shutdown();
			}

		}
		else 
		{//reach the destination
		// 4 Cases depending on the various scenarious and respective actions to be taken
			if(((current_x)<= (goal_x)) && ((current_y) <= (goal_y))) //Case1
			{
				printf("1st Case\n");
				goal = (atan((goal_y-current_y)/(goal_x-current_x)));
				goal_angle =  abs(angles::to_degrees(angles::normalize_angle(goal)));
			}
			else if(((current_x) >= (goal_x)) && ((current_y) <= (goal_y)))	//Case2
			{
				printf("2nd Case\n");
				goal = (atan((goal_x-current_x)/(goal_y-current_y)));
				goal_angle =  abs(angles::to_degrees(angles::normalize_angle(goal)))+90;				
			}
			if( ((current_x >= (goal_x)) && ((current_y) >= (goal_y)))) //Case3
			{
				printf("3rd Case\n");
				goal = (atan((goal_y-current_y)/(goal_x-current_x)));
				goal_angle =  abs(angles::to_degrees(angles::normalize_angle(goal)));
				goal_angle = goal_angle + 180.00; 		
			}
			else if((current_x < goal_x) && (current_y > goal_y)) //Case4
			{
				printf("4th Case\n");
				goal = (atan((goal_y-current_y)/(goal_x-current_x)));
				goal_angle =  abs(angles::to_degrees(angles::normalize_angle(goal)));
				goal_angle = 360- goal_angle;		
			}
			
			decideTurnDirection(goal_angle, theta_degree);

			printf("Goal Angle: %f Current Angle: %f \n", goal_angle, (theta_degree));

			diff = abs(round(goal_angle) - round(theta_degree));
			
			if (round(diff) != 0)
			{
			//align the robot
				if(clock_wise == true)
					motion.angular.z = -0.1;
				else
					motion.angular.z = 0.1;
		
				pub.publish(motion);
			}
			else
			{//go straight
				motion.linear.x = 0.2;
		
				pub.publish(motion);		
			}
		}
		
		ros::spinOnce();
		loop_rate.sleep();
	}	
	return 0;
}

