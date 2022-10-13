#include "ros/ros.h"
#include "std_msgs/String.h"

#include "ThrusterManager.hpp"

#include <sstream>

int main(int argc, char **argv) {
	ros::init(argc, argv, "thruster");

	ros::NodeHandle n;

	// ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

	ros::Rate loop_rate(10);

	int count = 0;
	while (ros::ok()) {

		// chatter_pub.publish(msg);

		ros::spinOnce();
		loop_rate.sleep();

		++count;
	}

	return 0;
}