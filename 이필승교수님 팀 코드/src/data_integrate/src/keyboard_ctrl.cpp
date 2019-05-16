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
#include "ros/ros.h"
#include <signal.h>
#include <ncurses.h>
#include <stdlib.h>
#include <string>

#include <std_msgs/Float32MultiArray.h>


using namespace std;

#define RAD2DEG(x) ((x)*180./M_PI)

#define PORT 3000
#define IPADDR "172.16.0.1"
//#define IPADDR "127.0.0.1" //my IP

int c_socket, s_socket;
struct sockaddr_in c_addr;
int len;
int n;

float data[24];

int sign;
int sign2;

static int line = 0;

static void logevent(const char* format, ...) {
	va_list va;

	move(9 + line, 0);
	clrtoeol();

	move(8 + line, 0);
	clrtoeol();

	va_start(va, format);
	vwprintw(stdscr, format, va);
	va_end(va);

	if (++line == 14) {
		line = 0;
	}
}

uint8_t tchar = 0xFF;

#define RAD2DEG(x) ((x)*180./M_PI)

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
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "keyboard_ctrl");
    ros::NodeHandle n;

    ros::Publisher msg_pub = n.advertise<std_msgs::Float32MultiArray>("/keyboard_ctrl", 1);
  	std_msgs::Float32MultiArrayPtr msg_ctrl_;
  	msg_ctrl_.reset(new std_msgs::Float32MultiArray);

    int ch;

    initscr();
  	cbreak();
  	noecho();
  	timeout(1);

    dataInit();

    c_socket = socket(PF_INET, SOCK_STREAM, 0);
    c_addr.sin_addr.s_addr = inet_addr(IPADDR);
    c_addr.sin_family = AF_INET;
    c_addr.sin_port = htons(PORT);

    if(connect(c_socket, (struct sockaddr*) &c_addr, sizeof(c_addr)) == -1){
        printf("Failed to connect\n");
        close(c_socket);
        return -1;
    }


    while(true)
    {
		    dataInit();
        sign = 0;
        sign2 = 0;
        ch = getch();


        while(true)
        {
        		if (ch == 's')
        		{
        			data[14]=1;
              sign2 = 1;
        		}
            else if (ch == 'w')
        		{
        			data[17]=1;
              sign2 = 1;
        		}
            else if (ch == 'a')
        		{
        			data[16]=1;
              sign2 = 1;
        		}
            else if (ch == 'd')
        		{
        			data[15]=1;
              sign2 = 1;
        		}

            ch = getch();

            if(sign2==1)
            break;
        }

        write(c_socket, data, sizeof(data));



        msg_ctrl_->data.clear();
    		msg_ctrl_->data.insert(msg_ctrl_->data.end(), data, data+24);
    		msg_pub.publish(msg_ctrl_);

        ros::Duration(0.01).sleep();
        ros::spinOnce();
  	    //ros::spinOnce();

    }
    close(c_socket);

  	ros::shutdown();

  	endwin();


    return 0;
}
