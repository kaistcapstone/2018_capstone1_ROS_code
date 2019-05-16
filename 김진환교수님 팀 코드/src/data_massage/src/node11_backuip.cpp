#include "ros/ros.h"
#include "data_massage/QTUM.h"
#include <stdio.h>
#include <algorithm>
#include <stdlib.h>
#include <ncurses.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <sensor_msgs/Imu.h>

using namespace std;

double siny;
double cosy;
double yaw = 0;
double q[4];

void msgCallback(const sensor_msgs::Imu::ConstPtr& msg1)
{
	q[0] = msg1->orientation.x;
	q[1] = msg1->orientation.y;
	q[2] = msg1->orientation.z;
	q[3] = msg1->orientation.w;

	// yaw (z-axis rotation)
	siny = +2.0 * (q[3] * q[2] + q[0] * q[1]);
	cosy = +1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]);
	yaw = atan2(siny, cosy);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node1");


  ros::NodeHandle n;
  data_massage::QTUM msg;
	ros::Subscriber node1_sub = n.subscribe("/imu", 1, msgCallback);


  ros::Publisher node1_pub = n.advertise<data_massage::QTUM>("yaw", 10);

  ros::Rate loop_rate(15);


  while (ros::ok())  {

		msg.data = yaw;
		node1_pub.publish(msg);

  	loop_rate.sleep();


    ros::spinOnce();

  }


	return 0;
}
