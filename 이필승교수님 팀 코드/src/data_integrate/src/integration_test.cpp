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
#include "sensor_msgs/LaserScan.h"

#include "opencv2/opencv.hpp"

using namespace std;

#define RAD2DEG(x) ((x)*180./M_PI)

int ball_number;
float ball_X[20];
float ball_Y[20];
float ball_distance[20];

float car_angle;
float car_to_ball_angle[20];
float car_to_ball_distance[20];

float car_fx = 0;
float car_fy = 0;
float car_bx = 0;
float car_by = 0;

float car_center_x = 0;
float car_center_y = 0;

float compare = 0;
float go_threshold1 = 0.9;
float go_threshold2 = 1.1;
float threshold_angle = 3;

int car_show = 1;

float data[24];


ros::Publisher pub;


void dataInit()
{
	data[0] = 0; //lx*data[3];
	data[1] = 0; //ly*data[3];
	data[2] = 0; //GamepadStickAngle(_dev, STICK_LEFT);
	data[3] = 0; //GamepadStickLength(_dev, STICK_LEFT);
	data[4] = 0; //rx*data[7];
	data[5] = 0; //ry*data[7];
	data[6] = 0; //GamepadStickAngle(_dev, STICK_RIGHT);
	data[7] = 0; //GamepadStickLength(_dev, STICK_RIGHT);
	data[8] = 0; //GamepadTriggerLength(_dev, TRIGGER_LEFT);
	data[9] = 0; //GamepadTriggerLength(_dev, TRIGGER_RIGHT);
	data[10] = 0; //GamepadButtonDown(_dev, BUTTON_DPAD_UP);
	data[11] = 0; //GamepadButtonDown(_dev, BUTTON_DPAD_DOWN);
	data[12] = 0; //GamepadButtonDown(_dev, BUTTON_DPAD_LEFT);
	data[13] = 0; //GamepadButtonDown(_dev, BUTTON_DPAD_RIGHT);
	data[14] = 0; //GamepadButtonDown(_dev, BUTTON_A); // duct on/off
	data[15] = 0; //GamepadButtonDown(_dev, BUTTON_B);
	data[16] = 0; //GamepadButtonDown(_dev, BUTTON_X);
	data[17] = 0; //GamepadButtonDown(_dev, BUTTON_Y);
	data[18] = 0; //GamepadButtonDown(_dev, BUTTON_BACK);
	data[19] = 0; //GamepadButtonDown(_dev, BUTTON_START);
	data[20] = 0; //GamepadButtonDown(_dev, BUTTON_LEFT_SHOULDER);
	data[21] = 0; //GamepadButtonDown(_dev, BUTTON_RIGHT_SHOULDER);
	data[22] = 0; //GamepadButtonDown(_dev, BUTTON_LEFT_THUMB);
	data[23] = 0; //GamepadButtonDown(_dev, BUTTON_RIGHT_THUMB);

	car_fx = 0;
	car_fy = 0;
	car_bx = 0;
	car_by = 0;
	car_center_x = 0;
	car_center_y = 0;

	car_angle = 0;
	for(int i=0; i<20; i++)
	{
		car_to_ball_angle[i] = 0;
		car_to_ball_distance[i] = 0;
	}
	compare = 0;
	car_show = 1;
}

void camera_Callback(const core_msgs::ball_position::ConstPtr& position)
{
    int count = position->size;
    ball_number=count;

		if(position->car_front_x.size()!=0)
    	car_fx = position->car_front_x[0];
		else{
			car_show = 0;
		}

		if(position->car_front_y.size()!=0)
    	car_fy = position->car_front_y[0];
		else{
			car_show = 0;
		}

		if(position->car_back_x.size()!=0)
    	car_bx = position->car_back_x[0];
		else{
			car_show = 0;
		}

		if(position->car_back_y.size()!=0)
    	car_by = position->car_back_y[0];
		else{
			car_show = 0;
		}

		if(car_show != 0)
		{
			car_center_x = (car_fx+car_bx)/2;
	    car_center_y = (car_fy+car_by)/2;
			car_angle = RAD2DEG(atan((car_fy - car_by) / (car_fx - car_bx)));

		}


    for(int i = 0; i < count; i++)
    {
        ball_X[i] = position->img_x[i];
        ball_Y[i] = position->img_y[i];
        // std::cout << "degree : "<< ball_degree[i];
        // std::cout << "   distance : "<< ball_distance[i]<<std::endl;
				// ball_distance[i] = ball_X[i]*ball_X[i]+ball_Y[i]*ball_X[i];
    }

		if(car_show != 0)
		{
			for(int i=0; i < count; i++)
			{
					car_to_ball_angle[i]=RAD2DEG(atan((ball_Y[i]-car_center_y)/(ball_X[i]-car_center_x)));
					car_to_ball_distance[i] = sqrt((ball_X[i] - car_center_x) * (ball_X[i] - car_center_x) + (ball_Y[i] - car_center_y) * (ball_Y[i] - car_center_y));
			}
		}


}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "integration_test_node");
    ros::NodeHandle n;

    //ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, lidar_Callback);
    ros::Subscriber sub1 = n.subscribe<core_msgs::ball_position>("/position", 1000, camera_Callback);
		pub = n.advertise<core_msgs::direction_indicator>("/direction_indicator", 100);

		core_msgs::direction_indicator dir;

    dataInit();

    while(ros::ok)
		{
				dataInit();
				ros::spinOnce();

				dir.straight_on.resize(ball_number);

				for(int i = 0; i < ball_number; i++)
				{
					std::cout << "ball_X"<< i << "=" << ball_X[i];
					std::cout << ", ball_Y"<< i << "=" << ball_Y[i] << std::endl;
				}

				if(car_show == 1)
				{
					cout << "Car center = " << car_center_x << "," << car_center_y << endl;
					cout << "Car vector = " << car_angle << endl;

					if(ball_number!=0)
					{
						for(int i = 0; i < ball_number; i++)
						{
							compare = car_angle - car_to_ball_angle[i];
							cout << "Car to ball " << i << " vector = " << car_to_ball_angle[i] << endl;

							if( abs(compare) < threshold_angle)
							{
								cout << "Car and ball " << i << " are in a line. " ;
								cout << "Go straight to ball " << i << "." << endl;
								cout << "Car - Ball " << i << " distance = " << car_to_ball_distance[i] << endl;
								dir.straight_on[i] = 1;
							}
							else{
								dir.straight_on[i] = 0;

								cout << "Car and ball " << i << " are 'NOT' in a line. " ;
								cout << "Car - Ball " << i << " distance = " << car_to_ball_distance[i] << endl;
								if (compare < -1 * threshold_angle)
								cout << "Turn RIGHT to ball " << i << "." << endl;
								else if(compare > threshold_angle)
								cout << "Turn LEFT to ball " << i << "." << endl;
							}
						}
					}
					else
					{
						cout << "no ball detected" << endl;
					}
				}
				else
				{
					cout << "No car is shown." << endl;
				}

				pub.publish(dir);
		    ros::Duration(0.1).sleep();
    }

    return 0;
}
