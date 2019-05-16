#include "ros/ros.h"
#include "data_massage/QTUM.h"
#include "data_massage/BTC.h"
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

int temp = 0;
float c = 0.8;
// float *myArray;
int myArray_len=0;

int *pix_x;
int *pix_y;
float *pix_r;

int max_index = 0;

void msgCallback(const core_msgs::ball_position::ConstPtr& msg1)
{
	// printf("Is it even working?\n");
	myArray_len = (int)msg1->size;
	// printf("arraylength: %d\n", myArray_len);
	// myArray = new float[myArray_len];
	pix_x = new int[myArray_len];
	pix_y = new int[myArray_len];
	pix_r = new float[myArray_len];
	// ROS_INFO("recieved from node1 msg = %d", msg1->stamp.sec);
	// ROS_INFO("recieved from node1 msgmsg = %d", msg->stamp.nsec);
	// printf("position info recieved\n");
	for(int i = 0; i < myArray_len; i++){
		// ROS_INFO("/position = (%d,%d,%.2f)\n", msg1->img_x[i],msg1->img_y[i],msg1->img_r[i]);
		pix_x[i] = msg1->img_x[i];
		pix_y[i] = msg1->img_y[i];
		pix_r[i] = msg1->img_r[i];
	}

	max_index = (max_element(pix_r, pix_r+myArray_len) - pix_r);
	// printf("position info ended\n");

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node2");


  ros::NodeHandle n;
  data_massage::BTC msg;
	ros::Subscriber node2_sub = n.subscribe("/position", 1, msgCallback);


  ros::Publisher node2_pub = n.advertise<data_massage::BTC>("BTCGAZUA", 1);

  ros::Rate loop_rate(20);




	// ros::spin();
  // ros::spinOnce();

  while (ros::ok())
  {

		// printf("\nmax element index: %d\n", max_index);
		// sort(pix_y,pix_y+myArray_len);
  	// msg.stamp = ros::Time::now();
		// printf("plz.... leng: %d\n", myArray_len);
		// for(int i = 0; i < myArray_len; i++){
		// 	printf("myArray[%d] = %d", i,myArray[i]);
		// }
		printf("arraylength: %d\n", myArray_len);
		msg.data = 0;
		msg.data2 =0;
		if (myArray_len >0){

			// msg.data = temp + c*(pix_x[max_index]-temp);
			msg.data = pix_x[max_index];
			msg.data2 = pix_y[max_index];

		}else{
			// printf("NO INFO or MAX\n");
		}
		temp = msg.data;
		// printf("%d\n",myArray[myArray_len-1]);
  	// msg.max = myArray[myArray_len-1];
		// printf("plz.... max: %d\n", msg.max);

  	// ROS_INFO("node2 send msg = %d", msg.stamp.sec);
  	// ROS_INFO("send msg = %d", msg.stamp.nsec);

		//seg errored ver
  	// ROS_INFO("node2 send msg = %d", msg.data);
		// if (myArray_len >0){
		// 	printf("%d: (%d,%d,%f)\n", max_index, pix_x[max_index],pix_y[max_index],pix_r[max_index]);
		// }
		// myArray_len = 0;
		// if (pix_r[max_index]>0){
  	// 	node2_pub.publish(msg);
		// }else {
		// 	printf("empty shell....\n");
		// }

		// for(int k=0; k<5;k++){
		if(myArray_len ==0){
			// printf("something has gone wrong\n");
			msg.real = 0;
		}
		if (myArray_len > 0){
			// printf("%d: (%d,%d,%f)\n", max_index, pix_x[max_index],pix_y[max_index],pix_r[max_index]);
			if (pix_r[max_index]>0){
				msg.data = pix_x[max_index];
				msg.data2 = pix_y[max_index];
				msg.real= 1;
				// ROS_INFO("node2 send msg = %d", msg.data);
				// node2_pub.publish(msg);
			}else {
				// printf("empty shell....\n");
				msg.real = 0;
			}
		}else{
			// printf("what\n");
		}


		node2_pub.publish(msg);
		myArray_len = 0;
		msg.real = 0;
		max_index=0;
  	loop_rate.sleep();
		// }

    // ++count;

    ros::spinOnce();

  }


	return 0;
}
