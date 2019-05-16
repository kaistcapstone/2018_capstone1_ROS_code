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
#include <vector>

using namespace std;

#define mindthegap 5
#define RANGE 160
#define resol_x 640
#define resol_y 480
#define RANGE2 440

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
// bool g_leftleft = false;
// bool g_rightright = false;
// int *pix_x;
// int *pix_y;
// float *pix_r;
// float *pix_x;
// float *pix_r;
// int max_index = 0;
// float** p;
int rbl = 0;
int lbr = 0;

bool sortcol( const vector<float>& v1,
               const vector<float>& v2 ) {
 return v1[0] < v2[0];
}
bool sortcol2( const vector<float>& v1,
               const vector<float>& v2 ) {
 return v1[0] > v2[0];
}

void msgCallback(const core_msgs::ball_position::ConstPtr& msg1)
{
	// printf("Is it even working?\n");
	rbl = 640;
	lbr = 0;
	int ball_len = 0;
	vector<vector<float> > v1d2;
	vector<vector<float> > v2d2;
	vector<float> upper_y;
	myArray_len = (int)msg1->size;
	for(int i = 0; i < myArray_len; i++){
		if(msg1->img_r[i]==0){
			continue;
		}
		ball_len++;
		vector<float> temp;
		vector<float> temp2;
		temp.push_back(msg1->img_x[i] - msg1->img_r[i]);
		temp.push_back(msg1->img_x[i] + msg1->img_r[i]);

		temp2.push_back(msg1->img_x[i] + msg1->img_r[i]);
		temp2.push_back(msg1->img_x[i] - msg1->img_r[i]);

		v1d2.push_back(temp);
		v2d2.push_back(temp);
		upper_y.push_back(msg1->img_y[i] - msg1->img_r[i]);
		// if(msg1->img_y[i]==0 || msg1->img_x[i] ==0){
		// 	continue;
		// }
		// if(msg1->img_y[i]>480 || msg1->img_x[i] > 640){
		// 	continue;
		// }

		if(msg1->img_r[i] !=0){
			g_exist = true;
		}

		// if(msg1->img_y[i] >445){
		// 	g_up = true;
		// }
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
		// if(msg1->img_x[i] < RANGE2){
		// 	g_leftleft = true;
		// }
		// if(msg1->img_x[i] > resol_x - RANGE2){
		// 	g_rightright = true;
		// }
	}
	//from the left
	// for(int i=0;i<v1d2.size(); i++) {
	// 	 for (int j=0;j<v1d2[i].size(); j++)
	// 		 cout << v2d2[i][j] << " ";
	// 	 cout << endl;
	// }



	//from the right
	if(ball_len>0){
		sort(v1d2.begin(), v1d2.end(), sortcol);
		lbr = v1d2[0][1];
		sort(v2d2.begin(), v2d2.end(), sortcol2);
		sort(upper_y.begin(),upper_y.end());
		// rbr = v2d2[0][0];
		rbl = v2d2[0][1];
		if(upper_y[0] > RANGE2){
			g_up = true;
		}
		// cout<<"//////   " << upper_y[0] <<"    //////" <<endl;
	}


	for(int i = 0; i<ball_len; i++){
		if(lbr >= v1d2[i][0]){
			lbr = v1d2[i][1];
		}else{
			break;
		}
	}


	for(int i=0; i<ball_len; i++){
		if (rbl <= v2d2[i][1]){
			rbl = v2d2[i][0];
		}else{
			break;
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
		msg.lbr = lbr;
		msg.rbl = rbl;
		// msg.ll = g_leftleft;
		// msg.rr = g_rightright;


		node2_pub.publish(msg);

		g_left = false;
		g_right = false;
		g_up = false;
		g_exist = false;
		g_left2 = false;
		g_right2 = false;
		// g_leftleft = false;
		// g_rightright = false;
  	loop_rate.sleep();
		// }
		cout<< "lbr: " <<lbr <<" // rbl: "<<rbl <<endl;

    // ++count;

    ros::spinOnce();

  }


	return 0;
}
