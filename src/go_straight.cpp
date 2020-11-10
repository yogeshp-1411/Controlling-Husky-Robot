#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "go_straight");
	ros::NodeHandle n;
	
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/twist_marker_server/cmd_vel", 1000);

	ros::Rate loop_rate(10);
	
	while (ros::ok())
	{
		geometry_msgs::Twist motion;
		
		motion.linear.x = 0.2;
		motion.linear.y = 0.0;
		motion.linear.z = 0.0;
		motion.angular.x = 0.0;
		motion.angular.y = 0.0;		
		motion.angular.z = 0.0;
		
		pub.publish(motion);

		ros::spinOnce();
		loop_rate.sleep();
	}	
	return 0;
}
