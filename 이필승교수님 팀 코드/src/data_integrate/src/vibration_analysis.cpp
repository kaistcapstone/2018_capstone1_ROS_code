#include <iostream>
#include <stdio.h>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <string>
#include <math.h>
#include <stdlib.h>
#include <string.h>


#include <ros/ros.h>
#include <ros/package.h>
#include "core_msgs/ball_position.h"


#include "opencv2/opencv.hpp"

using namespace std;


float car_fx = 0;
float car_fy = 0;
float car_bx = 0;
float car_by = 0;
int car_show = 0;

void camera_Callback(const core_msgs::ball_position::ConstPtr& position)
{

  car_show = position->car_front_x.size();

  if(position->car_front_x.size()!=0)
		car_fx = position->car_front_x[0];
  else
    car_fx = 0;

  if(position->car_front_y.size()!=0)
		car_fy = position->car_front_y[0];
  else
    car_fy = 0;

    if(position->car_back_x.size()!=0)
  		car_bx = position->car_back_x[0];
    else
      car_bx = 0;

      if(position->car_back_y.size()!=0)
    		car_by = position->car_back_y[0];
      else
        car_by = 0;



	}

  int main(int argc, char **argv)
  {
      ros::init(argc, argv, "vibration_analysis");
      ros::NodeHandle n;

      ros::Subscriber sub1 = n.subscribe<core_msgs::ball_position>("/position", 1000, camera_Callback);
      ros::spinOnce();
      while(car_show == 0)
      {
        ros::spinOnce();
      }
      cout << "car_fx / car_fy / car_bx / car_by" << endl;
      ros::Duration(1).sleep();

      for(int i; i<1000; i++)
      {
        cout << car_fx << " " << car_fy << " " << car_bx << " " << car_by << endl;
        ros::Duration(0.02).sleep();
        ros::spinOnce();
      }



      return 0;
  }
