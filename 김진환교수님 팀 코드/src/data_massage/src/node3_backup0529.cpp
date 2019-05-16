#include "ros/ros.h"
#include "data_massage/QTUM.h"
#include "data_massage/BTC.h"
#include "data_massage/ETHER.h"
#include <stdio.h>
#include <algorithm>
#include <stdlib.h>

// #include <math.h>

// #include <stdio.h>
#include <ncurses.h>
// #include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include "core_msgs/ball_position.h"
#include <math.h>

using namespace std;

#define mindthegap 5
#define RANGE 160
#define resol_x 640
#define resol_y 480
#define RANGE2 40

int temp = 0;
// float c = 0.8;
// float *myArray;
int myArray_len=0;

bool g_left = false;
bool g_right = false;
bool g_up = false;
bool g_exist = false;
bool g_left2 = false;
bool g_right2 = false;
bool g_leftleft = false;
bool g_rightright = false;
// int *pix_x;
// int *pix_y;
// float *pix_r;

// int max_index = 0;

void msgCallback(const core_msgs::ball_position::ConstPtr& msg1)
{
	// printf("Is it even working?\n");

	myArray_len = (int)msg1->size;
	for(int i = 0; i < myArray_len; i++){
		if(msg1->img_y[i]==0 || msg1->img_x[i] ==0){
			continue;
		}
		if(msg1->img_y[i]>480 || msg1->img_x[i] > 640){
			continue;
		}

		if(msg1->img_r[i] !=0){
			g_exist = true;
		}

		if(msg1->img_y[i] >445){
			g_up = true;
		}
		if(msg1->img_x[i] >= resol_x/2 -RANGE && msg1->img_x[i] < resol_x/2-RANGE + mindthegap ){
			g_left2 = true;
		}

		if(msg1->img_x[i] < resol_x/2 - RANGE && !g_left){
			// printf("leftball?\n");
			g_left = true;
		}
		if(msg1->img_x[i] <= resol_x/2 + RANGE && msg1->img_x[i] > resol_x/2+RANGE - mindthegap){
			g_right2 = true;
		}
		if(msg1->img_x[i] > resol_x/2 + RANGE && !g_right){
			// printf("rightball?\n");
			g_right = true;
		}
		if(msg1->img_x[i] < RANGE2){
			g_leftleft = true;
		}
		if(msg1->img_x[i] > resol_x - RANGE2){
			g_rightright = true;
		}
	}

	// printf("position info ended\n");

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node3");


  ros::NodeHandle n;
  data_massage::ETHER msg;
	ros::Subscriber node2_sub = n.subscribe("/posg", 1, msgCallback);


  ros::Publisher node2_pub = n.advertise<data_massage::ETHER>("ETHERGAZUA", 1);

  ros::Rate loop_rate(20);




	// ros::spin();
  // ros::spinOnce();

  while (ros::ok())
  {


		msg.left = g_left;
		msg.right = g_right;
		msg.above = g_up;
		msg.exist = g_exist;
		msg.left2 = g_left2;
		msg.right2= g_right2;
		msg.ll = g_leftleft;
		msg.rr = g_rightright;


		node2_pub.publish(msg);

		g_left = false;
		g_right = false;
		g_up = false;
		g_exist = false;
		g_left2 = false;
		g_right2 = false;
		g_leftleft = false;
		g_rightright = false;
  	loop_rate.sleep();
		// }

    // ++count;

    ros::spinOnce();

  }


	return 0;
}
