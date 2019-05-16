#include "ros/ros.h"
#include "capstone7_bonus/BTC.h"

void msgCallback(const capstone7_bonus::BTC::ConstPtr& msg)
{
	ROS_INFO("recieve msg = %d", msg->stamp.sec);
	// ROS_INFO("recieve msg = %d", msg->stamp.nsec);
	ROS_INFO("recieve msg = %d", msg->data);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "node3");
	ros::NodeHandle n;

	ros::Subscriber node3_sub = n.subscribe("BTCGAZUA", 100, msgCallback);

	ros::spin();

	return 0;
}
