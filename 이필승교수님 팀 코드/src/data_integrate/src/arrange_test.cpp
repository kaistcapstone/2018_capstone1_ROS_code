#include <iostream>
#include <stdio.h>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <string>
#include <signal.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <ros/ros.h>

#include <ros/ros.h>
#include <ros/package.h>
#include "core_msgs/ball_position.h"
#include "core_msgs/direction_indicator.h"
#include "ros/ros.h"

#include "opencv2/opencv.hpp"

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arrange_test");


  int count = 8;
  float ball_X[]={13.2,11.3,6,7,8.7,9,15,1,0,0,0,0};
  float ball_Y[]={13.2,11.3,6,7,8.7,9,15,1,0,0,0,0};
  float car_to_ball_distance[8];

  for(int i=0; i<8;i++)
  {
    car_to_ball_distance[i] = ball_X[i]*ball_X[i] + ball_Y[i]*ball_Y[i];
  }

  float smallest_distance = 0;
  float arranged_ball_order_x[8];
  float arranged_ball_order_y[8];

  int i, j, indexMin, temp = 0;

    for (i = 0; i < count - 1; i++)
    {
        indexMin = i;
        for (j = i + 1; j < count; j++)
        {
            if (car_to_ball_distance[j] < car_to_ball_distance[indexMin])
            {
                indexMin = j;
            }
        }
        temp = ball_X[indexMin];
        ball_X[indexMin] = ball_X[i];
        ball_X[i] = temp;

        temp = ball_Y[indexMin];
        ball_Y[indexMin] = ball_Y[i];
        ball_Y[i] = temp;

        temp = car_to_ball_distance[indexMin];
        car_to_ball_distance[indexMin] = car_to_ball_distance[i];
        car_to_ball_distance[i] = temp;
    }

  for(int i = 0; i<8; i++)
  {
    cout << ball_X[i] << ", " << ball_Y[i] << endl;
  }

  return 0;
}
