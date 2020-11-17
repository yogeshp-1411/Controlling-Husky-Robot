#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "math.h"
#include "tf/tf.h"
#include "angles/angles.h"

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

	//printf("In the callback Current_x: %f	Current_y: %f\n", current_x, current_y);

	tf::Quaternion q( msg->pose.pose.orientation.x,
		 	msg->pose.pose.orientation.y,
			msg->pose.pose.orientation.z,
			msg->pose.pose.orientation.w);

	double theta_yaw = tf::getYaw(q);
	//theta_degree = angles::to_degrees(angles::normalize_angle_positive(theta_yaw));
	theta_degree = round(angles::to_degrees(angles::normalize_angle_positive(theta_yaw)));
}

//function to decide the turn direction of the robot. 
// true:-> turn clockwise, 
// false :-> turn anticlockwise
void decideTurnDirection(double goal_angle, double current_angle)
{
	if((goal_angle - current_angle ) > -120 && (goal_angle - current_angle ) <= 0)
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

		//printf("Diff_x: %f 	Diff_y: %f\n", (goal_x - current_x), (goal_y - current_y));
		printf("Diff_x: %f 	Diff_y: %f\n", round(goal_x - current_x), round(goal_y - current_y));
		printf("Current_x: %f	Current_y: %f\n", current_x, current_y);

		//if( (((goal_x - current_x) ==0)&& ((goal_y - current_y) ==0)) || ((round(goal_x - current_x) == 0)&&((goal_y - current_y) ==0))||(((goal_x - current_x) == 0) &&(round(goal_y - current_y) ==0)))
		if ( (round(goal_x - current_x) == 0) && (round(goal_y - current_y) == 0) )
		{//once reached the destination, align the robot in x direction
			printf("STOPPING :::: Current Angle %f\n", theta_degree);
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
		else //if((round(goal_x - current_x) !=0) || (round(goal_y - current_y)!=0))
		{//reach the destination
			if(((current_x)<= (goal_x)) && ((current_y) <= (goal_y))) //Case1
			{
				printf("1st Case\n");
				goal = (atan((goal_y-current_y)/(goal_x-current_x)));
				//goal_angle =  angles::to_degrees(angles::normalize_angle_positive(goal));
				goal_angle =  abs(angles::to_degrees(angles::normalize_angle(goal)));
				//clock_wise = false;				
			}
			else if(((current_x) >= (goal_x)) && ((current_y) <= (goal_y)))	//Case2
			{
				printf("2nd Case\n");
				goal = (atan((goal_x-current_x)/(goal_y-current_y)));
				//printf("goal angle positive %f\n", angles::to_degrees(angles::normalize_angle_positive(goal)));
				//printf("goal angle normalized %f\n", angles::to_degrees(angles::normalize_angle(goal)));
				goal_angle =  abs(angles::to_degrees(angles::normalize_angle(goal)))+90;
				//clock_wise = false;									
			}
			//else if(( ((current_x >= (goal_x)) && ((current_y) >= (goal_y))) || ( ((current_x >= (goal_x)) && ((current_y) >= (goal_y))))
			if( ((current_x >= (goal_x)) && ((current_y) >= (goal_y))) || ( ((round(current_x) >= (goal_x)) && ((current_y) >= (goal_y)))) )
			{
				printf("3rd Case\n");
				goal = (atan((goal_y-current_y)/(goal_x-current_x)));
				//goal_angle =  angles::to_degrees(angles::normalize_angle_positive(goal))+180;
				//printf("goal angle normalized %f\n", angles::to_degrees(angles::normalize_angle(goal)));
				goal_angle =  abs(angles::to_degrees(angles::normalize_angle(goal)));
				goal_angle = goal_angle + 180.00; 
//printf("GOAL ANGLE %f\n", goal_angle);
				//clock_wise = false;							
			}
			else if((current_x < goal_x) && (current_y > goal_y))
			{
				printf("4th Case\n");
				goal = (atan((goal_y-current_y)/(goal_x-current_x)));
				//printf("goal without normalization %f\n", goal);
				//goal_angle =  angles::to_degrees(angles::normalize_angle_positive(goal));	
				goal_angle =  abs(angles::to_degrees(angles::normalize_angle(goal)));
				goal_angle = 360- goal_angle;	
				//clock_wise = true;		
			}
			
			decideTurnDirection(goal_angle, theta_degree);

			//goal = (atan(goal_y/goal_x));
			//goal = (atan((goal_y-current_y)/(goal_x-current_x)));

			//goal_angle =  angles::to_degrees(angles::normalize_angle_positive(goal)); //was used
			//goal_angle =  angles::to_degrees(angles::normalize_angle(goal));
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

