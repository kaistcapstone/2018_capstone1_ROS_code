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
#include "core_msgs/station_msg.h"
#include "ros/ros.h"

#include "opencv2/opencv.hpp"

using namespace std;

#define RAD2DEG(x) ((x)*180./M_PI)

#define PORT 3000
#define IPADDR "172.16.0.1"
//#define IPADDR "127.0.0.1"

int ball_number;
int ball_number_saved;
float ball_X[10];
float ball_Y[10];
float ball_distance[10];

float car_angle;
float car_to_ball_angle[10];
float car_to_ball_distance[10];
float arranged_ball_order_x[10];
float arranged_ball_order_y[10];
float car_to_home_angle = 0;

float car_fx = 0;
float car_fy = 0;
float car_bx = 0;
float car_by = 0;
float car_length = 0;
float car_center_x = 0;
float car_center_y = 0;

float compare = 0;
int data_compare = 0;
float go_threshold1 = 0.8;
float go_threshold2 = 1.2;
float threshold_angle = 4;
float stop_time = 0.25;

float distance_threshold, init_distance_threshold = 20;
float door_open_threshold, init_door_open_threshold = 100;
float forced_go_distance, init_forced_go_distance = 120;
float door_open_ratio = 0;
float forced_go_ratio = 0;

float smallest_distance = 0;
int forced_go = 0;

float destination_x=0;
float destination_y=0;
float destination_x_array[3];
float destination_y_array[3];
float destination_distance;

int car_show = 1;
int station_state;
int step = 0;
int initial_step = 0;
int next_step = 0;
float delay = 0.02;
float delay2 = 0.02;

float data[6];
float backup_data[6];

float map_x = 640;
float map_y = 480;

int c_socket, s_socket;
struct sockaddr_in c_addr;
int len;
int n;

ros::Publisher pub;

float angle_difference(float a, float b)
{
	if((a-b)>180)
	{
		return 360-(a-b);
	}
	else if(a-b<-180)
	{
		return a-b+360;
	}
	else{
		return a-b;
	}
}

float marker_calibration(float y)
{


	float output = 1.1033*y + 15.473;
	return output;

}

float marker_calibration_f(float y)
{


	float output = 1.0783*y + 15.44;
	return output;

}

float threshold_calibration(float init_threshold, float car_value)
{
	float output = init_threshold * (1 - 0.5*(map_y-car_value)/map_y);
	return output;
}

// float threshold_calibration_door(float init_threshold, float car_value, float car_angle_value)
// {
// 	if(car_angle_value >180 && car_angle_value <= 270)
// 		car_angle_value = car_angle_value - 180;
// 	else if(car_angle_value > 270)
// 		car_angle_value = 360 - car_angle_value;
// 	else if(car_angle_value > 90 && car_angle_value <= 180)
// 		car_angle_value = 180-car_angle_value;
//
// 	float output = init_threshold * (1 - 0.2*(map_y-car_value)/map_y-0.8*car_angle_value/90);
// 	return output;
// }



float threshold_calibration_forced(float init_threshold, float car_value)
{
	float output = init_threshold * (1 - 0.75*(map_y-car_value)/map_y);
	return output;
}


void dataInit()
{
	data[0] = 0;
	data[1] = 0;
	data[2] = 0;
	data[3] = 0;
	data[4] = 0;
	data[5] = 0;

	backup_data[0] = 0;
	backup_data[1] = 0;
	backup_data[2] = 0;
	backup_data[3] = 0;
	backup_data[4] = 0;
	backup_data[5] = 0;


	car_show = 1;

}


void station_Callback(const core_msgs::station_msg::ConstPtr& msg)
{
	station_state = msg->station_state;
}

void camera_Callback(const core_msgs::ball_position::ConstPtr& position)
{

	int count = position->size;
	ball_number=count/2;



	if(position->car_front_x.size()!=0)
		car_fx = position->car_front_x[0];
	else{
		car_show = 0;
	}

	if(position->car_front_y.size()!=0)
	{
		car_fy = position->car_front_y[0];
		car_fy =  marker_calibration_f(car_fy);
	}
	else{
		car_show = 0;
	}

	if(position->car_back_x.size()!=0)
		car_bx = position->car_back_x[0];
	else{
		car_show = 0;
	}

	if(position->car_back_y.size()!=0)
	{
		car_by = position->car_back_y[0];
		car_by =  marker_calibration(car_by);
	}
	else{
		car_show = 0;
	}

	if(car_show != 0)
	{

		car_center_x = car_fx;
		car_center_y = car_fy;
		car_length = sqrt ((car_bx-car_fx)*(car_bx-car_fx) + (car_by-car_fy)*(car_by-car_fy));
		car_angle = RAD2DEG(atan((car_fy - car_by) / (car_fx - car_bx)));
		if(car_fx < car_bx)
		{
			if(car_angle<0)
			{
				car_angle = car_angle + 360;
			}
		}
		else if(car_fx > car_bx)
		{
			car_angle = car_angle + 180;
		}
	}


	for(int i = 0; i < ball_number; i++)
	{
			ball_X[i] = position->img_x[2*i];
			ball_Y[i] = position->img_y[2*i];

	}

	if(car_show != 0)
	{
		for(int i=0; i < ball_number; i++)
		{
			car_to_ball_angle[i]=RAD2DEG(atan((ball_Y[i]-car_center_y)/(ball_X[i]-car_center_x)));

			car_to_ball_distance[i] = sqrt( (ball_X[i] - car_center_x) * (ball_X[i] - car_center_x) + (ball_Y[i] - car_center_y) * (ball_Y[i] - car_center_y) );

		}


		int i, j, indexMin = 0;
		float temp = 0;

    for (i = 0; i < ball_number - 1; i++)
    {
        indexMin = i;
        for (j = i + 1; j < ball_number; j++)
        {
            if (ball_Y[j] > ball_Y[indexMin])
            {
                indexMin = j;
            }
        }

        temp = ball_X[indexMin];
        ball_X[indexMin] = ball_X[i];
        ball_X[i] = temp;
				arranged_ball_order_x[i] = temp;

        temp = ball_Y[indexMin];
        ball_Y[indexMin] = ball_Y[i];
        ball_Y[i] = temp;
				arranged_ball_order_y[i] = temp;

        temp = car_to_ball_distance[indexMin];
        car_to_ball_distance[indexMin] = car_to_ball_distance[i];
        car_to_ball_distance[i] = temp;
    }
		arranged_ball_order_x[ball_number-1] = ball_X[ball_number-1];
		arranged_ball_order_y[ball_number-1] = ball_Y[ball_number-1];

		for(int i=0; i < ball_number; i++)
		{
			car_to_ball_angle[i]=RAD2DEG(atan((arranged_ball_order_y[i]-car_center_y)/(arranged_ball_order_x[i]-car_center_x)));

			if(arranged_ball_order_x[i] < car_center_x )
			{
				if(car_to_ball_angle[i]<0)
				{
					car_to_ball_angle[i] = car_to_ball_angle[i] + 360;
				}
			}
			else if(arranged_ball_order_x[i] > car_center_x )
			{
				car_to_ball_angle[i] = car_to_ball_angle[i] + 180;
			}

			car_to_ball_distance[i] = sqrt( (arranged_ball_order_x[i] - car_bx) * (arranged_ball_order_x[i] - car_bx) + (arranged_ball_order_y[i] - car_by) * (arranged_ball_order_y[i] - car_by) );
		}
	}

}



void go_straight()
{
	data[0] = 0; //lx*data[3];
	data[1] = 0.45; //ly*data[3];
	data[2] = 0; //GamepadStickAngle(_dev, STICK_LEFT);
	data[3] = 0; //GamepadStickLength(_dev, STICK_LEFT);
	data[4] = 0; //rx*data[7];
	data[5] = 0; //ry*data[7];
}

void go_straight_start()
{
	data[0] = 0; //lx*data[3];
	data[1] = 0.25; //ly*data[3];
	data[2] = 0; //GamepadStickAngle(_dev, STICK_LEFT);
	data[3] = 0; //GamepadStickLength(_dev, STICK_LEFT);
	data[4] = 0; //rx*data[7];
	data[5] = 0; //ry*data[7];
}

void go_straight_forced()
{
	data[0] = 0; //lx*data[3];
	data[1] = 0.3; //ly*data[3];
	data[2] = 0; //GamepadStickAngle(_dev, STICK_LEFT);
	data[3] = 0; //GamepadStickLength(_dev, STICK_LEFT);
	data[4] = 0; //rx*data[7];
	data[5] = 0; //ry*data[7];
}

void go_back()
{
	data[0] = 0; //lx*data[3];
	data[1] = -0.3; //ly*data[3];
	data[2] = 0; //GamepadStickAngle(_dev, STICK_LEFT);
	data[3] = 0; //GamepadStickLength(_dev, STICK_LEFT);
	data[4] = 0; //rx*data[7];
	data[5] = 0; //ry*data[7];
}

void go_back_slow()
{
	data[0] = 0; //lx*data[3];
	data[1] = -0.2; //ly*data[3];
	data[2] = 0; //GamepadStickAngle(_dev, STICK_LEFT);
	data[3] = 0; //GamepadStickLength(_dev, STICK_LEFT);
	data[4] = 0; //rx*data[7];
	data[5] = 0; //ry*data[7];
}

void go_back_continuously(int y)
{
	data[0] = 0; //lx*data[3];
	data[1] = -0.25-0.4 * (float(y)) / 300; //ly*data[3];
	data[2] = 0; //GamepadStickAngle(_dev, STICK_LEFT);
	data[3] = 0; //GamepadStickLength(_dev, STICK_LEFT);
	data[4] = 0; //rx*data[7];
	data[5] = 0; //ry*data[7];
}

void go_back_continuously2(int y)
{
	data[0] = 0; //lx*data[3];
	data[1] = -0.7+0.65 * min(float(1), (float(y)) / 400); //ly*data[3];
	data[2] = 0; //GamepadStickAngle(_dev, STICK_LEFT);
	data[3] = 0; //GamepadStickLength(_dev, STICK_LEFT);
	data[4] = 0; //rx*data[7];
	data[5] = 0; //ry*data[7];
}



void turn_left(float compare)
{

	data[0] = 0; //lx*data[3];
	data[1] = 0; //ly*data[3];
	data[2] = 0.13+3*pow((min(fabs(compare),float(90))/90),2); //GamepadStickAngle(_dev, STICK_LEFT);
	data[3] = 0; //GamepadStickLength(_dev, STICK_LEFT);
	data[4] = 0; //rx*data[7];
	data[5] = 0; //ry*data[7];
	cout << "compare & pow((abs(compare)/180),3/4) &  data[2] = " << compare <<   " & " << data[2] << endl;
}

void turn_left_slow(float compare)
{

	data[0] = 0; //lx*data[3];
	data[1] = 0; //ly*data[3];
	data[2] = 0.1+1.5*pow((min(fabs(compare),float(90))/90),2); //GamepadStickAngle(_dev, STICK_LEFT);
	data[3] = 0; //GamepadStickLength(_dev, STICK_LEFT);
	data[4] = 0; //rx*data[7];
	data[5] = 0; //ry*data[7];
	cout << "compare & pow((abs(compare)/180),3/4) &  data[2] = " << compare <<   " & " << data[2] << endl;
}

void turn_right(float compare)
{
	data[0] = 0; //lx*data[3];
	data[1] = 0; //ly*data[3];
	data[2] = 0; //GamepadStickAngle(_dev, STICK_LEFT);
	data[3] = 0.13+3*pow((min(fabs(compare),float(90))/90),2); //GamepadStickLength(_dev, STICK_LEFT);
	data[4] = 0; //rx*data[7];
	data[5] = 0; //ry*data[7];
	cout << "compare & data[3] = " << compare <<  " & " << data[3] << endl;

}

void turn_right_slow(float compare)
{
	data[0] = 0; //lx*data[3];
	data[1] = 0; //ly*data[3];
	data[2] = 0; //GamepadStickAngle(_dev, STICK_LEFT);
	data[3] = 0.1+1.5*pow((min(fabs(compare),float(90))/90),2); //GamepadStickLength(_dev, STICK_LEFT);
	data[4] = 0; //rx*data[7];
	data[5] = 0; //ry*data[7];
	cout << "compare & data[3] = " << compare <<  " & " << data[3] << endl;

}

void go_left(float gap)
{
	data[0] = -0.05-0.2*pow((min(fabs(gap),float(80))/80), 2); //lx*data[3];
	data[1] = 0; //ly*data[3];
	data[2] = 0; //GamepadStickAngle(_dev, STICK_LEFT);
	data[3] = 0; //GamepadStickLength(_dev, STICK_LEFT);
	data[4] = 0; //rx*data[7];
	data[5] = 0; //ry*data[7];
	cout << "gap & data[0] = " << gap << " & "  << data[0] << endl;

}

void go_left_fast(float gap)
{
	data[0] = -0.15-0.3*pow((min(fabs(gap),float(80))/80), 2); //lx*data[3];
	data[1] = 0; //ly*data[3];
	data[2] = 0; //GamepadStickAngle(_dev, STICK_LEFT);
	data[3] = 0; //GamepadStickLength(_dev, STICK_LEFT);
	data[4] = 0; //rx*data[7];
	data[5] = 0; //ry*data[7];
	cout << "gap & data[0] = " << gap << " & "  << data[0] << endl;

}

void go_left2()
{
	data[0] = -0.3; //lx*data[3];
	data[1] = 0; //ly*data[3];
	data[2] = 0; //GamepadStickAngle(_dev, STICK_LEFT);
	data[3] = 0; //GamepadStickLength(_dev, STICK_LEFT);
	data[4] = 0; //rx*data[7];
	data[5] = 0; //ry*data[7];


}

void go_right(float gap)
{
	data[0] = 0.05+0.2*pow((min(fabs(gap),float(80))/80), 2); //lx*data[3];
	data[1] = 0; //ly*data[3];
	data[2] = 0; //GamepadStickAngle(_dev, STICK_LEFT);
	data[3] = 0; //GamepadStickLength(_dev, STICK_LEFT);
	data[4] = 0; //rx*data[7];
	data[5] = 0; //ry*data[7];
	cout << "gap & data[0] = " << gap <<  " & " << data[0] << endl;
}

void go_right_fast(float gap)
{
	data[0] = 0.15+0.3*pow((min(fabs(gap),float(80))/80), 2); //lx*data[3];
	data[1] = 0; //ly*data[3];
	data[2] = 0; //GamepadStickAngle(_dev, STICK_LEFT);
	data[3] = 0; //GamepadStickLength(_dev, STICK_LEFT);
	data[4] = 0; //rx*data[7];
	data[5] = 0; //ry*data[7];
	cout << "gap & data[0] = " << gap <<  " & " << data[0] << endl;
}

void go_right2()
{
	data[0] = 0.3; //lx*data[3];
	data[1] = 0; //ly*data[3];
	data[2] = 0; //GamepadStickAngle(_dev, STICK_LEFT);
	data[3] = 0; //GamepadStickLength(_dev, STICK_LEFT);
	data[4] = 0; //rx*data[7];
	data[5] = 0; //ry*data[7];

}



void stop()
{
	data[0] = 0; //lx*data[3];
	data[1] = 0; //ly*data[3];
	data[2] = 0; //GamepadStickAngle(_dev, STICK_LEFT);
	data[3] = 0; //GamepadStickLength(_dev, STICK_LEFT);
	data[4] = 0; //rx*data[7];
	data[5] = 0; //ry*data[7];
}



void door_open()
{

	data[4] = 1;
	data[5] = 0;
}

void door_close()
{
	data[4] = 0;
	data[5] = 1;
}

void door_close_final()
{
	data[4] = -0.3;
	data[5] = 0;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_integation");
    ros::NodeHandle n;

    ros::Subscriber sub1 = n.subscribe<core_msgs::ball_position>("/position", 1000, camera_Callback);
		ros::Subscriber sub2 = n.subscribe<core_msgs::station_msg>("/station_state",100, station_Callback);
		pub = n.advertise<core_msgs::direction_indicator>("/direction_indicator", 100);

		core_msgs::ball_position msg;
		core_msgs::station_msg msg2;
		core_msgs::direction_indicator dir;

    c_socket = socket(PF_INET, SOCK_STREAM, 0);
    c_addr.sin_addr.s_addr = inet_addr(IPADDR);
    c_addr.sin_family = AF_INET;
    c_addr.sin_port = htons(PORT);

		dataInit();

    if(connect(c_socket, (struct sockaddr*) &c_addr, sizeof(c_addr)) == -1){
        printf("Failed to connect\n");
        close(c_socket);
        return -1;
    }

		step = 0;
		next_step = 1;
		int a = 0;
		station_state = 0;
		car_show = 0;


		ros::Duration(3.5).sleep();


		ros::spinOnce();


		cout << "Operation Start." << endl;

		if(step!=3)
		{
			while(car_show==0 || station_state == 0) // || station_state == 0
			{
				car_show = 1;
				cout << "Go straight until car is shown." << endl;
				for(int i=0; i<10 ; i++)
				{
					go_straight_start();

					write(c_socket, data, sizeof(data));
					ros::Duration(0.02).sleep();
				}
				ros::spinOnce();
			}

			door_close();
			write(c_socket, data, sizeof(data));
		}



		if(step!=3)
		{
			while(ball_number==0)
			{
				cout << "Camera on." << endl;
				ros::spinOnce();
				for(int i=0; i<3; i++)
				{
					destination_x_array[i] = arranged_ball_order_x[i];
					destination_y_array[i] = arranged_ball_order_y[i];
				}
				initial_step = ball_number;

			}
			ros::spinOnce();
			initial_step = ball_number;
		}
		else
		{
			initial_step = 3;
		}



    while(step<initial_step)
		{

				ros::spinOnce();




				std::cout << "## We are now in Step "<< step << ". ##" << endl;


				if(car_show == 1)
				{
					cout << "Car center = " << car_center_x << "," << car_center_y << endl;
					cout << "Car vector = " << car_angle << endl;
					cout << "Ball number =  " << ball_number << endl;

					if(ball_number!=0 || destination_x !=0)
					{
						////


							cout << "Ball detected." << endl;
							cout << "next_step = " << next_step << endl;

							if(next_step == 1)
							{
								ros::spinOnce();
								destination_x = arranged_ball_order_x[0];
								destination_y = arranged_ball_order_y[0];


								next_step = 0;
							}
							cout << "destination x =  " << destination_x << endl;
							cout << "destination y =  " << destination_y << endl;


							destination_distance = sqrt((destination_x - car_center_x) * (destination_x - car_center_x) + (destination_y - car_center_y) * (destination_y - car_center_y) );
							car_to_ball_angle[step] = RAD2DEG(atan((destination_y-car_fy)/(destination_x-car_fx)));

							if(destination_x < car_fx )
							{
								if(car_to_ball_angle[step]<0)
								{
									car_to_ball_angle[step] = car_to_ball_angle[step] + 360;
								}
							}
							else if(destination_x > car_fx )
							{
								car_to_ball_angle[step] = car_to_ball_angle[step] + 180;
							}


							compare = angle_difference(car_angle, car_to_ball_angle[step]);
							cout << "Car to ball " << step << " vector = " << car_to_ball_angle[step] << endl;
							cout << "Car - Ball " << step << " distance = " << destination_distance << endl;
							cout << "Compare angle to ball " << step << " = " << compare << endl;

							forced_go_distance = threshold_calibration(init_forced_go_distance, car_center_y);

							if(forced_go==1)
							{
								////////////
								forced_go_distance = threshold_calibration(init_forced_go_distance, car_center_y);
								distance_threshold = threshold_calibration(init_distance_threshold, car_center_y);


								stop();
								write(c_socket, data, sizeof(data));
								ros::Duration(1).sleep();
								ros::spinOnce();

								if( compare > 1)
								{

									while(compare > 0)
									{

										cout << "Car and ball " << step << " are NOT in a line. " ;
										cout << "Operate turn left." << endl;

										std::cout << "ball_X"<< step << "=" << destination_x;
										std::cout << ", ball_Y"<< step << "=" << destination_y << std::endl;

										cout << "Car - Ball " << step << " distance = " << destination_distance << endl;
										cout << "Car vector = " << car_angle << endl;
										cout << "Car to ball " << step << " vector = " << car_to_ball_angle[step] << endl;
										cout << "Compare angle to ball " << step << " = " << compare << endl;
										cout << car_angle <<", "<< car_to_ball_angle[step] << endl;
										turn_left(compare);

										write(c_socket, data, sizeof(data));

										ros::Duration(delay).sleep();
										ros::spinOnce();

										car_to_ball_angle[step] = RAD2DEG(atan((destination_y-car_fy)/(destination_x-car_fx)));
										if(destination_x < car_fx )
										{
											if(car_to_ball_angle[step]<0)
											{
												car_to_ball_angle[step] = car_to_ball_angle[step] + 360;
											}
										}
										else if(destination_x > car_fx )
										{
											car_to_ball_angle[step] = car_to_ball_angle[step] + 180;
										}

										compare = angle_difference(car_angle, car_to_ball_angle[step]);


									}
								}

							 else if(compare < -1)
								{
									while(compare < 0)
									{

										cout << "Car and ball " << step << " are NOT in a line. " ;
										cout << "Operate turn right." << endl;

										std::cout << "ball_X"<< step << "=" << destination_x;
										std::cout << ", ball_Y"<< step << "=" << destination_y << std::endl;

										cout << "Car - Ball " << step << " distance = " << destination_distance << endl;
										cout << "Car vector = " << car_angle << endl;
										cout << "Car to ball " << step << " vector = " << car_to_ball_angle[step] << endl;
										cout << "Compare angle to ball " << step << " = " << compare << endl;
										cout << car_angle <<", "<< car_to_ball_angle[step] << endl;
										turn_right(compare);

										write(c_socket, data, sizeof(data));

										ros::Duration(delay).sleep();
										ros::spinOnce();

										car_to_ball_angle[step] = RAD2DEG(atan((destination_y-car_fy)/(destination_x-car_fx)));
										if(destination_x < car_fx )
										{
											if(car_to_ball_angle[step]<0)
											{
												car_to_ball_angle[step] = car_to_ball_angle[step] + 360;
											}
										}
										else if(destination_x > car_fx )
										{
											car_to_ball_angle[step] = car_to_ball_angle[step] + 180;
										}

										compare = angle_difference(car_angle, car_to_ball_angle[step]);


									}
								}
								////////////

								int forced_step = 0;

								if(car_angle >180 && car_angle <= 270)
										car_angle = car_angle - 180;
									else if(car_angle > 270)
										car_angle = 360 - car_angle;
									else if(car_angle > 90 && car_angle <= 180)
										car_angle = 180-car_angle;

								door_open_ratio = -0.003 * car_angle + 0.8177;

								while(forced_step<150)
								{
									cout << "Car and ball are close. Go straight." << endl ;
									cout << "Car to ball distance = " << destination_distance << endl ;
									go_straight();

									if(destination_distance <= door_open_ratio * car_length * 1.8)
									{
										door_open();
									}
									write(c_socket, data, sizeof(data));
									ros::Duration(delay).sleep();
									ros::spinOnce();

									forced_step++;


									distance_threshold = threshold_calibration(init_distance_threshold, car_center_y);


									destination_distance = sqrt((destination_x - car_center_x) * (destination_x - car_center_x) + (destination_y - car_center_y) * (destination_y - car_center_y) );

									if(destination_distance <= distance_threshold)
									{
										cout << "Car arrived at ball " << step << "." << endl ;

										step++;
										next_step = 1;

										for(int i=0; i<15; i++)
										{
											go_straight_forced();
											write(c_socket, data, sizeof(data));
											ros::Duration(0.02).sleep();
										}
										for(int i=0; i<10; i++)
										{
											go_straight_forced();
											door_close();
											write(c_socket, data, sizeof(data));
											ros::Duration(0.02).sleep();
										}



										ros::spinOnce();

										if(car_show == 0)
										{
											cout << "Too much go. Proceed go back." << endl;
											while(car_show==0)
											{
												car_show = 1;
												for(int i=0; i<25 ; i++)
												{
													go_back();
													write(c_socket, data, sizeof(data));
													ros::Duration(0.02).sleep();
												}
												ros::spinOnce();
											}
										}


										stop();
										door_close();

										write(c_socket, data, sizeof(data));
										ros::Duration(stop_time).sleep();
										forced_go = 0;
										next_step = 1;
										break;
									}
								}

								door_close();
								write(c_socket, data, sizeof(data));
								ros::Duration(delay).sleep();
							}

							else if( abs(compare) <= threshold_angle)
							{


								while(compare <= threshold_angle+10 && compare >= -1 * threshold_angle -10)
								{
									destination_distance = sqrt( (destination_x - car_center_x) * (destination_x - car_center_x) + (destination_y - car_center_y) * (destination_y - car_center_y) );
									car_to_ball_angle[step] = RAD2DEG(atan((destination_y-car_fy)/(destination_x-car_fx)));

									if(destination_x < car_fx )
									{
										if(car_to_ball_angle[step]<0)
										{
											car_to_ball_angle[step] = car_to_ball_angle[step] + 360;
										}
									}
									else if(destination_x > car_fx )
									{
										car_to_ball_angle[step] = car_to_ball_angle[step] + 180;
									}

									compare = angle_difference(car_angle, car_to_ball_angle[step]);

									/////
									float car_angle2 = 0;
									if(car_angle >180 && car_angle <= 270)
											car_angle2 = car_angle - 180;
										else if(car_angle > 270)
											car_angle2 = 360 - car_angle;
										else if(car_angle > 90 && car_angle <= 180)
											car_angle2 = 180-car_angle;

									forced_go_ratio = -0.003 * car_angle2 + 0.8177;

									forced_go_distance = forced_go_ratio * car_length * 1.85;
									////////


									std::cout << "## We are now in Step "<< step << ". ##" << endl;
									cout << "Car and ball " << step << " are in a line. " ;
									cout << "Go straight to ball " << step << "." << endl;

									std::cout << "ball_X"<< step << "=" << destination_x;
									std::cout << ", ball_Y"<< step << "=" << destination_y << std::endl;

									cout << "Car - Ball " << step << " distance = " << destination_distance << endl;
									cout << "Car vector = " << car_angle << endl;
									cout << "Car to ball " << step << " vector = " << car_to_ball_angle[step] << endl;
									cout << "Compare angle to ball " << step << " = " << compare << endl;

									go_straight();

									if(destination_distance < forced_go_distance && compare <= threshold_angle)
									{
										forced_go = 1;
										break;
									}


									write(c_socket, data, sizeof(data));


									ros::Duration(delay).sleep();
									ros::spinOnce();




									car_to_ball_angle[step] = RAD2DEG(atan((destination_y-car_fy)/(destination_x-car_fx)));
									compare = angle_difference(car_angle, car_to_ball_angle[step]);




								}
							}

						 	else if( compare > threshold_angle)
							{

								while(compare > 0)
								{

									std::cout << "## We are now in Step "<< step << ". ##" << endl;
									cout << "Car and ball " << step << " are NOT in a line. " ;
									cout << "Operate turn left." << endl;

									std::cout << "ball_X"<< step << "=" << destination_x;
									std::cout << ", ball_Y"<< step << "=" << destination_y << std::endl;

									cout << "Car - Ball " << step << " distance = " << destination_distance << endl;
									cout << "Car vector = " << car_angle << endl;
									cout << "Car to ball " << step << " vector = " << car_to_ball_angle[step] << endl;
									cout << "Compare angle to ball " << step << " = " << compare << endl;
									cout << car_angle <<", "<< car_to_ball_angle[step] << endl;
									turn_left(compare);

									destination_distance = sqrt((destination_x - car_center_x) * (destination_x - car_center_x) + (destination_y - car_center_y) * (destination_y - car_center_y) );

									/////
									float car_angle2 = 0;
									if(car_angle >180 && car_angle <= 270)
											car_angle2 = car_angle - 180;
										else if(car_angle > 270)
											car_angle2 = 360 - car_angle;
										else if(car_angle > 90 && car_angle <= 180)
											car_angle2 = 180-car_angle;

									forced_go_ratio = -0.003 * car_angle2 + 0.8177;

									forced_go_distance = forced_go_ratio * car_length * 1.85;
									////////

									if(destination_distance<=forced_go_distance && abs(compare) < threshold_angle)
									{
										forced_go=1;
										break;
									}


									write(c_socket, data, sizeof(data));

									ros::Duration(delay).sleep();
									ros::spinOnce();


									car_to_ball_angle[step] = RAD2DEG(atan((destination_y-car_fy)/(destination_x-car_fx)));
									if(destination_x < car_fx )
									{
										if(car_to_ball_angle[step]<0)
										{
											car_to_ball_angle[step] = car_to_ball_angle[step] + 360;
										}
									}
									else if(destination_x > car_fx )
									{
										car_to_ball_angle[step] = car_to_ball_angle[step] + 180;
									}

									compare = angle_difference(car_angle, car_to_ball_angle[step]);



								}
							}

						 else if(compare < -1 * threshold_angle)
							{
								while(compare < 0)
								{
									std::cout << "## We are now in Step "<< step << ". ##" << endl;
									cout << "Car and ball " << step << " are NOT in a line. " ;
									cout << "Operate turn right." << endl;

									std::cout << "ball_X"<< step << "=" << destination_x;
									std::cout << ", ball_Y"<< step << "=" << destination_y << std::endl;

									cout << "Car - Ball " << step << " distance = " << destination_distance << endl;
									cout << "Car vector = " << car_angle << endl;
									cout << "Car to ball " << step << " vector = " << car_to_ball_angle[step] << endl;
									cout << "Compare angle to ball " << step << " = " << compare << endl;
									cout << car_angle <<", "<< car_to_ball_angle[step] << endl;
									turn_right(compare);

									destination_distance = sqrt((destination_x - car_center_x) * (destination_x - car_center_x) + (destination_y - car_center_y) * (destination_y - car_center_y) );

									/////
									float car_angle2 = 0;
									if(car_angle >180 && car_angle <= 270)
											car_angle2 = car_angle - 180;
										else if(car_angle > 270)
											car_angle2 = 360 - car_angle;
										else if(car_angle > 90 && car_angle <= 180)
											car_angle2 = 180-car_angle;

									forced_go_ratio = -0.003 * car_angle2 + 0.8177;

									forced_go_distance = forced_go_ratio * car_length * 1.85;
									////////

									if(destination_distance<=forced_go_distance && abs(compare) < threshold_angle)
									{
										forced_go = 1;
										break;
									}



									write(c_socket, data, sizeof(data));



									ros::Duration(delay).sleep();
									ros::spinOnce();


									car_to_ball_angle[step] = RAD2DEG(atan((destination_y-car_fy)/(destination_x-car_fx)));
									if(destination_x < car_fx )
									{
										if(car_to_ball_angle[step]<0)
										{
											car_to_ball_angle[step] = car_to_ball_angle[step] + 360;
										}
									}
									else if(destination_x > car_fx )
									{
										car_to_ball_angle[step] = car_to_ball_angle[step] + 180;
									}

									compare = angle_difference(car_angle, car_to_ball_angle[step]);


								}
							}
							else
							{
								cout << "Error" << endl;
							}



							if(step==3)
							{
								stop();
								write(c_socket, data, sizeof(data));
								ros::Duration(stop_time).sleep();

								for(int i=0; i<50; i++)
								{
									go_back_slow();
									write(c_socket, data, sizeof(data));
									ros::Duration(0.02).sleep();
								}

								stop();
								write(c_socket, data, sizeof(data));
								ros::Duration(0.5).sleep();

								ros::spinOnce();
								if(ball_number != 0)
								{
									cout << "Some of balls are not catched. Repeat procedure." << endl ;
									ros::Duration(stop_time).sleep();
									step--;
									next_step = 1;
								}
								else{
									destination_x = 0;
									destination_y = 0;
								}
							}

							ros::spinOnce();




					}
					else
					{
						cout << "no ball detected" << endl;
						next_step = 1;
					}
				}
				else
				{
					cout << "No car is shown." << endl;
					ros::spinOnce();
				}



				//pub.publish(dir);
				//write(c_socket, data, sizeof(data));
		    //ros::Duration(1).sleep();


    }







		if(step>=initial_step)
		{
			while(step<initial_step+1)
			{
				cout << "Proceed return home" << endl;
				ros::Duration(stop_time).sleep();
				ros::spinOnce();
				if(car_angle<0)
				{
					car_angle = 180+car_angle;
				}

				if(car_center_x <= map_x / 2)
				{
					if(car_fx<car_bx)
					{
						while(car_angle < 90)
						{
							cout << "Proceed turn right." << endl;
							turn_right((90-car_angle));

							write(c_socket, data, sizeof(data));

							ros::Duration(delay).sleep();
							ros::spinOnce();
							if(car_angle<0)
							{
								car_angle = 180+car_angle;
							}
						}
					}
					else if(car_fx>car_bx)
					{
						while(car_angle > 90)
						{
							cout << "Proceed turn left." << endl;
							turn_left((car_angle-90));

							write(c_socket, data, sizeof(data));


							ros::Duration(delay).sleep();
							ros::spinOnce();
							if(car_angle<0)
							{
								car_angle = 180+car_angle;
							}
						}
					}

					stop();
					write(c_socket, data, sizeof(data));
					ros::Duration(stop_time).sleep();

					while(car_center_x < map_x / 2)
					{
						cout << "Proceed go right." << endl;
						go_right_fast(map_x/2 - car_center_x);

						write(c_socket, data, sizeof(data));


						ros::Duration(delay2).sleep();
						ros::spinOnce();
					}

				}

				else if(car_center_x > map_x/2 )
				{
					if(car_fx<car_bx)
					{
						while(car_angle < 90)
						{
							cout << "Proceed turn right." << endl;
							turn_right((90-car_angle));


							write(c_socket, data, sizeof(data));

							ros::Duration(delay).sleep();
							ros::spinOnce();
							if(car_angle<0)
							{
								car_angle = 180+car_angle;
							}
						}
					}
					else if(car_fx>car_bx)
					{
						while(car_angle > 90)
						{
							cout << "Proceed turn left." << endl;
							turn_left((car_angle-90));

							write(c_socket, data, sizeof(data));


							ros::Duration(delay).sleep();
							ros::spinOnce();
							if(car_angle<0)
							{
								car_angle = 180+car_angle;
							}
						}
					}

					stop();
					write(c_socket, data, sizeof(data));
					ros::Duration(stop_time).sleep();

					while(car_center_x > map_x/2)
					{cout << "Proceed go left." << endl;
						go_left_fast(car_center_x-map_x/2);

						write(c_socket, data, sizeof(data));

						ros::Duration(delay2).sleep();
						ros::spinOnce();
					}

				}

				//////
				if( car_angle < 89 || car_angle > 91)
				{
					if(car_fx<car_bx)
					{
						while(car_angle < 90)
						{
							cout << "Proceed turn right." << endl;
							turn_right((90-car_angle)/2);


							write(c_socket, data, sizeof(data));

							ros::Duration(delay).sleep();
							ros::spinOnce();
							if(car_angle<0)
							{
								car_angle = 180+car_angle;
							}
						}
					}
					else if(car_fx>car_bx)
					{
						while(car_angle > 90)
						{
							cout << "Proceed turn left." << endl;
							turn_left((car_angle-90)/2);

							write(c_socket, data, sizeof(data));


							ros::Duration(delay).sleep();
							ros::spinOnce();
							if(car_angle<0)
							{
								car_angle = 180+car_angle;
							}
						}
					}
					else{

					}
				}

				/////

				ros::spinOnce();
				if(car_angle<0)
				{
					car_angle = 180+car_angle;
				}

				// for(int i=0; i<25; i++)
				// {
				// 	go_left2();
				// 	write(c_socket, data, sizeof(data));
				// 	ros::Duration(0.02).sleep();
				// }
				// stop();
				// write(c_socket, data, sizeof(data));
				// ros::Duration(0.5).sleep();
				//
				// for(int i=0; i<50; i++)
				// {
				// 	go_right2();
				// 	write(c_socket, data, sizeof(data));
				// 	ros::Duration(0.02).sleep();
				// }
				// stop();
				// write(c_socket, data, sizeof(data));
				// ros::Duration(0.5).sleep();
				//
				//
				// for(int i=0; i<25; i++)
				// {
				// 	go_left2();
				// 	write(c_socket, data, sizeof(data));
				// 	ros::Duration(0.02).sleep();
				// }
				// stop();
				// write(c_socket, data, sizeof(data));
				// ros::Duration(0.5).sleep();

				stop();
				write(c_socket, data, sizeof(data));
				ros::Duration(stop_time).sleep();

				int back_step = 0;

				while(car_by<map_y-200)
				{


					destination_distance = sqrt((map_x/2 - car_bx) * (map_x/2 - car_bx) + (map_y - car_by) * (map_y - car_by) );
					cout << "Proceed go back." << endl;
					cout << "Car to home vector = " << car_to_home_angle << endl;
					cout << "Car - home distance = " << destination_distance << endl;
					go_back_continuously2(back_step);
					write(c_socket, data, sizeof(data));
					ros::Duration(delay).sleep();

					back_step++;



					///////////////////////////////////////////////////
					//////////////////////////////////////////////////
					// if( car_angle >= 85 && car_angle <= 95)
					// {
					//
					// 	while(destination_distance>distance_threshold)
					// 	{
					// 		destination_distance = sqrt((map_x/2 - car_bx) * (map_x/2 - car_bx) + (map_y - car_by) * (map_y - car_by) );
					//
					//
					// 		if(car_angle < 85 || car_angle > 95)
					// 		break;
					//
					// 		cout << "Car and home are in a line. " ;
					// 		cout << "Go straight to home." << endl;
					//
					// 		cout << "Car - home distance = " << destination_distance << endl;
					// 		cout << "Car vector = " << car_angle << endl;
					//
					// 		go_back();
					//
					//
					// 		write(c_socket, data, sizeof(data));
					//
					//
					// 		// destination_distance = sqrt((map_x/2 - car_bx) * (map_x/2 - car_bx) + (map_y - car_by) * (map_y -	 car_by) );
					// 		// if(destination_distance<distance_threshold)
					// 		// 	break;
					//
					//
					// 		ros::Duration(delay).sleep();
					// 		ros::spinOnce();
					//
					// 		if(car_angle<0)
					// 		{
					// 			car_angle = 180+car_angle;
					// 		}
					//
					// 		if(car_center_x<map_x/2-10)
					// 		{
					// 			while(car_center_x < map_x/2)
					// 			{
					// 				go_right(map_x/2-car_center_x);
					//
					//
					// 				write(c_socket, data, sizeof(data));
					//
					//
					// 				ros::Duration(0.05).sleep();
					// 				ros::spinOnce();
					// 			}
					// 		}
					// 		else if(car_center_x > map_x/2 +10)
					// 		{
					// 			while(car_center_x > map_x/2)
					// 			{
					// 				go_left(car_center_x - map_x/2);
					//
					//
					// 				write(c_socket, data, sizeof(data));
					//
					//
					// 				ros::Duration(0.05).sleep();
					// 				ros::spinOnce();
					// 				if(car_angle<0)
					// 				{
					// 					car_angle = 180+car_angle;
					// 				}
					// 			}
					// 		}
					//
					// 	}
					// }
					//
					// else if( car_angle < 85)
					// {
					// 	while(car_angle < 90)
					// 	{
					//
					// 		cout << "Car and home are NOT in a line. " ;
					// 		cout << "Operate turn right." << endl;
					//
					// 		cout << "Car - home distance = " << destination_distance << endl;
					// 		cout << "Car vector = " << car_angle << endl;
					// 		turn_right((90-car_angle)*2);
					//
					// 		write(c_socket, data, sizeof(data));
					//
					//
					// 		ros::Duration(delay).sleep();
					// 		ros::spinOnce();
					// 		if(car_angle<0)
					// 		{
					// 			car_angle = 180+car_angle;
					// 		}
					//
					//
					//
					//
					//
					// 	}
					// }
					//
					// else if(car_angle > 95)
					// {
					// 	while(car_angle >90)
					// 	{
					// 		cout << "Car and home are NOT in a line. " ;
					// 		cout << "Operate turn left." << endl;
					//
					// 		cout << "Car - home distance = " << destination_distance << endl;
					// 		cout << "Car vector = " << car_angle << endl;
					// 		turn_left(2*(car_angle-90));
					//
					//
					// 		write(c_socket, data, sizeof(data));
					//
					//
					// 		ros::Duration(delay).sleep();
					// 		ros::spinOnce();
					// 		if(car_angle<0)
					// 		{
					// 			car_angle = 180+car_angle;
					// 		}
					//
					// 	}
					// }
					/////////////////////////////////////////////////
					/////////////////////////////////////////////////
					ros::spinOnce();

				}
				stop();
				write(c_socket, data, sizeof(data));
				ros::Duration(stop_time).sleep();

				// if( car_angle < 85 || car_angle > 95)
				// {
				// 	if(car_fx<car_bx)
				// 	{
				// 		while(car_angle < 90)
				// 		{
				// 			turn_right((90-car_angle)*2);
				//
				//
				// 				write(c_socket, data, sizeof(data));
				//
				// 			ros::Duration(delay).sleep();
				// 			ros::spinOnce();
				// 			if(car_angle<0)
				// 			{
				// 				car_angle = 180+car_angle;
				// 			}
				// 		}
				// 	}
				// 	else if(car_fx>car_bx)
				// 	{
				// 		while(car_angle > 90)
				// 		{
				// 			turn_left(2*(car_angle-90));
				//
				// 				write(c_socket, data, sizeof(data));
				//
				//
				// 			ros::Duration(delay).sleep();
				// 			ros::spinOnce();
				// 			if(car_angle<0)
				// 			{
				// 				car_angle = 180+car_angle;
				// 			}
				// 		}
				// 	}
				// 	else
				// 	{
				//
				// 	}
				// }

				///////////////////////////////////////
				//////////////////////////////////////////////////




				cout << "Proceed Go to Center." << endl;



				if(car_center_x <= map_x / 2)
				{
					if(car_fx<car_bx)
					{
						while(car_angle < 90)
						{
							turn_right((90-car_angle));

							write(c_socket, data, sizeof(data));

							ros::Duration(delay2).sleep();
							ros::spinOnce();
							if(car_angle<0)
							{
								car_angle = 180+car_angle;
							}
						}
					}
					else if(car_fx>car_bx)
					{
						while(car_angle > 90)
						{
							turn_left((car_angle-90));

							write(c_socket, data, sizeof(data));


							ros::Duration(delay2).sleep();
							ros::spinOnce();
							if(car_angle<0)
							{
								car_angle = 180+car_angle;
							}
						}
					}

					stop();
					write(c_socket, data, sizeof(data));
					ros::Duration(stop_time).sleep();

					while(car_center_x <= map_x / 2)
					{
						go_right(map_x/2 - car_center_x);

							write(c_socket, data, sizeof(data));


						ros::Duration(delay2).sleep();
						ros::spinOnce();
					}

				}

				else if(car_center_x > map_x/2 )
				{
					if(car_fx<car_bx)
					{
						while(car_angle <= 90)
						{
							turn_right((90-car_angle));


							write(c_socket, data, sizeof(data));

							ros::Duration(delay2).sleep();
							ros::spinOnce();
							if(car_angle<0)
							{
								car_angle = 180+car_angle;
							}
						}
					}
					else if(car_fx>car_bx)
					{
						while(car_angle >= 90)
						{
							turn_left((car_angle-90));

							write(c_socket, data, sizeof(data));


							ros::Duration(delay2).sleep();
							ros::spinOnce();
							if(car_angle<0)
							{
								car_angle = 180+car_angle;
							}
						}
					}

					stop();
					write(c_socket, data, sizeof(data));
					ros::Duration(stop_time).sleep();

					while(car_center_x >= map_x/2)
					{
						go_left(car_center_x-map_x/2);

						write(c_socket, data, sizeof(data));

						ros::Duration(delay2).sleep();
						ros::spinOnce();
					}

				}


				if( car_angle < 89 || car_angle > 91)
				{
					if(car_fx<car_bx)
					{
						while(car_angle < 89.5)
						{
							turn_right((90-car_angle));


							write(c_socket, data, sizeof(data));

							ros::Duration(delay).sleep();
							ros::spinOnce();
							if(car_angle<0)
							{
								car_angle = 180+car_angle;
							}
						}
					}
					else if(car_fx>car_bx)
					{
						while(car_angle > 90.5)
						{
							turn_left((car_angle-90));

							write(c_socket, data, sizeof(data));


							ros::Duration(delay).sleep();
							ros::spinOnce();
							if(car_angle<0)
							{
								car_angle = 180+car_angle;
							}
						}
					}
				}

				stop();
				write(c_socket, data, sizeof(data));
				ros::Duration(stop_time).sleep();


				//////////////////// Red Ball cleaning

				//for(int i=0; i<40; i++)
				//{
				//	go_left2();
				//	write(c_socket, data, sizeof(data));
				//	ros::Duration(0.02).sleep();
				//}
				//stop();
				//write(c_socket, data, sizeof(data));
				//ros::Duration(0.5).sleep();

				//for(int i=0; i<80; i++)
				//{
				//	go_right2();
				//	write(c_socket, data, sizeof(data));
				//	ros::Duration(0.02).sleep();
				//}
				//stop();
				//write(c_socket, data, sizeof(data));
				//ros::Duration(0.5).sleep();


				//for(int i=0; i<40; i++)
				//{
				//	go_left2();
				//	write(c_socket, data, sizeof(data));
				//	ros::Duration(0.02).sleep();
				//}
				//stop();
				//write(c_socket, data, sizeof(data));
				//ros::Duration(0.5).sleep();

				ros::spinOnce();

				///////////////////// Make Center again
				////////////////////
				// if(car_center_x <= map_x / 2)
				// {
				// 	if(car_fx<car_bx)
				// 	{
				// 		while(car_angle < 90)
				// 		{
				// 			turn_right((90-car_angle));
				//
				// 			write(c_socket, data, sizeof(data));
				//
				// 			ros::Duration(delay2).sleep();
				// 			ros::spinOnce();
				// 			if(car_angle<0)
				// 			{
				// 				car_angle = 180+car_angle;
				// 			}
				// 		}
				// 	}
				// 	else if(car_fx>car_bx)
				// 	{
				// 		while(car_angle > 90)
				// 		{
				// 			turn_left((car_angle-90));
				//
				// 			write(c_socket, data, sizeof(data));
				//
				//
				// 			ros::Duration(delay2).sleep();
				// 			ros::spinOnce();
				// 			if(car_angle<0)
				// 			{
				// 				car_angle = 180+car_angle;
				// 			}
				// 		}
				// 	}
				//
				// 	stop();
				// 	write(c_socket, data, sizeof(data));
				// 	ros::Duration(0.5).sleep();
				//
				// 	while(car_center_x <= map_x / 2)
				// 	{
				// 		go_right(map_x/2 - car_center_x);
				//
				// 			write(c_socket, data, sizeof(data));
				//
				//
				// 		ros::Duration(delay2).sleep();
				// 		ros::spinOnce();
				// 	}
				//
				// }
				//
				// else if(car_center_x > map_x/2 )
				// {
				// 	if(car_fx<car_bx)
				// 	{
				// 		while(car_angle <= 90)
				// 		{
				// 			turn_right((90-car_angle));
				//
				//
				// 			write(c_socket, data, sizeof(data));
				//
				// 			ros::Duration(delay2).sleep();
				// 			ros::spinOnce();
				// 			if(car_angle<0)
				// 			{
				// 				car_angle = 180+car_angle;
				// 			}
				// 		}
				// 	}
				// 	else if(car_fx>car_bx)
				// 	{
				// 		while(car_angle >= 90)
				// 		{
				// 			turn_left((car_angle-90));
				//
				// 			write(c_socket, data, sizeof(data));
				//
				//
				// 			ros::Duration(delay2).sleep();
				// 			ros::spinOnce();
				// 			if(car_angle<0)
				// 			{
				// 				car_angle = 180+car_angle;
				// 			}
				// 		}
				// 	}
				//
				// 	stop();
				// 	write(c_socket, data, sizeof(data));
				// 	ros::Duration(0.5).sleep();
				//
				// 	while(car_center_x >= map_x/2)
				// 	{
				// 		go_left(car_center_x-map_x/2);
				//
				// 		write(c_socket, data, sizeof(data));
				//
				// 		ros::Duration(delay2).sleep();
				// 		ros::spinOnce();
				// 	}
				//
				// }
				//
				if( car_angle < 89 || car_angle > 91)
				{
					if(car_fx<car_bx)
					{
						while(car_angle < 89.5)
						{
							turn_right_slow((90-car_angle));


							write(c_socket, data, sizeof(data));

							ros::Duration(delay).sleep();
							ros::spinOnce();
							if(car_angle<0)
							{
								car_angle = 180+car_angle;
							}
						}
					}
					else if(car_fx>car_bx)
					{
						while(car_angle > 90.5)
						{
							turn_left_slow((car_angle-90));

							write(c_socket, data, sizeof(data));


							ros::Duration(delay).sleep();
							ros::spinOnce();
							if(car_angle<0)
							{
								car_angle = 180+car_angle;
							}
						}
					}
				}
				//////////////////
				stop();
				door_close_final();
				write(c_socket, data, sizeof(data));
				ros::Duration(0.5).sleep();

				for(int i=0; i<200; i++)
				{
					go_back_continuously(i);
					write(c_socket, data, sizeof(data));
					ros::Duration(0.02).sleep();
				}




				stop();
				write(c_socket, data, sizeof(data));

				step++;
				cout << "Operation finished." << endl;

			}


		}

    return 0;
}
