#include <stdio.h>
#include <ncurses.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include "core_msgs/ball_position.h"
#include <math.h>
#include "data_massage/QTUM.h"
#include "data_massage/BTC.h"
#include "data_massage/ETHER.h"
#include <fstream>
#include <iostream>


extern "C" {
	#include "xbox_ctrl/gamepad.h"
}

using namespace std;

#define PORT 4010
#define IPADDR "172.16.0.1"
// #define IPADDR "127.0.0.1"
#define resol_x 640
#define resol_y 480
#define strafe_max 1
#define strafe_min 0.3
#define turn_max 0.6
#define turn_min 0.05
#define RANGE_DEAD1 80
#define RANGE_DEAD2 30
#define RANGE_STATE_CHANGE 300
#define RANGE_TURN 0.3


// ofstream outfile("log.txt");

ofstream outfile;

static const char* button_names[] = {
	"d-pad up",
	"d-pad down",
	"d-pad left",
	"d-pad right",
	"start",
	"back",
	"left thumb",
	"right thumb",
	"left shoulder",
	"right shoulder",
	"???",
	"???",
	"A",
	"B",
	"X",
	"Y"
};

bool Blue_left = false;
bool Idle_right = true;


bool green_u = false;
bool green_l = false;
bool green_r = false;
bool green_e = false;
bool green_l2 = false;
bool green_r2 = false;


bool gohome = false;
bool flag_error = false;
bool flag_final = false;
bool flag_final1 = false;
bool flag_final2 =false;
bool flag_final3 = false;

int c_socket, s_socket;
struct sockaddr_in c_addr;
int len;

float data[24];
float old_data[24];
float fake = 0.0;
static int line = 0;

double yaw_init = 0.0;

double yaw_now = 0.0;
double yaw_old = 0.0;
double yaw_temp= 0.0;
double no_ball = 0.0;
void dataInit();


int n=0;
static void Calib(){
	n++;
	if(n==1){
		yaw_init = 0.0;
		yaw_temp = 0.0;
	}


	yaw_init = yaw_init + yaw_temp /(n+0.0);
}

static void FinalOri(){
	dataInit();

	if(yaw_temp>0 && yaw_temp < M_PI - 0.1){
		data[4] = -0.2;
	}else if(yaw_temp <=0 && yaw_temp > -1.0*M_PI + 0.1){
		data[4] = 0.2;
	}else if(yaw_temp <= M_PI){
		data[4] = -0.2 *(M_PI - yaw_temp);
	}else if(yaw_temp >= -1.0*M_PI){
		data[4] = 0.2 * (yaw_temp + M_PI);

	}
}
static void Orient(){
	dataInit();



	if (yaw_temp > RANGE_TURN){
		data[4] =turn_max*(yaw_temp)/3.14;
	}
	else if(yaw_temp < -1.0*RANGE_TURN){
		data[4] =  turn_max*(yaw_temp)/3.14;
	}
	else{
		data[4] = 0.2*yaw_temp;
	}
	// data[4] = 0.131;

	write(c_socket,data,sizeof(data));
	ros::Duration(0.05).sleep();
}



// bool home_ori = false;


static void Phase2Ori(){


	if(yaw_temp < 0.12 && yaw_temp >-0.12){
		// home_ori = true;
		flag_error = false;
	}
	data[4] = 0.15*yaw_temp;

	green_e = false;

	// if(!home_ori){
	// 	green_e = false;
	// 	if (yaw_temp > RANGE_TURN){
	// 		data[4] =turn_max*(yaw_temp)/3.14;
	// 	}
	// 	else if(yaw_temp < -1.0*RANGE_TURN){
	// 		data[4] = turn_max*(yaw_temp)/3.14;
	// 	}else{
	// 		data[4] = 0.1*yaw_temp;
	// 	}
	// }

}
static void Phase2(){
	dataInit();
	if(!green_e){
		if(Idle_right){
			data[2] = 0;
			data[3] = 0.111;
		}else{
			data[2] = M_PI;
			data[3] = 0.111;
		}
	}
	else{

		if(green_l && green_r){
			//proceed slowly
			if(!green_u){
				data[2] = M_PI/2.0;
				data[3] = strafe_max /4.0;
			}else{
				flag_final3 = true;
			}

		}else if(!green_l && !green_r){
			//proceed fast
			data[2] = M_PI/2.0;
			data[3] = strafe_max/2.0;
		}else if(!green_l && green_r){
			//go right
			// data[2] = 0;
			// data[3] = 0.13;
			// Idle_right = true;
			if(green_l2){
				data[2] = M_PI/2.0;
				data[3] = strafe_max /4.0;
			}else{
				data[2] = 0;
				data[3] = 0.13;
			}
		}else if(green_l && !green_r){

			if(green_r2){
				data[2] = M_PI/2.0;
				data[3] = strafe_max /4.0;
			}else{
				data[2] = M_PI-0.0001;
				data[3] = 0.13;
			}
			//go left

			// Idle_right = false;

		}
	}
	outfile << "GOING HOME: " << green_e << green_l << green_r << green_u << endl;
	outfile << "||" << data[2] << "||" <<data[3] << endl;
	// data[2] = atan2(data[1],data[0]);
	// data[3] =

}


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


static void update(GAMEPAD_DEVICE dev) {

	float lx, ly, rx, ry;
	move(dev, 0);

	if (!GamepadIsConnected(dev)) {
		printw("%d) n/a\n", dev);
		return;
	}

	GamepadStickNormXY(dev, STICK_LEFT, &lx, &ly);
	GamepadStickNormXY(dev, STICK_RIGHT, &rx, &ry);

	printw("%d) L:(%+.3f,%+.3f :: %+.3f,%+.3f) R:(%+.3f, %+.3f :: %+.3f,%+.3f) LT:%+.3f RT:%+.3f ",
		dev,
		lx, ly,
		GamepadStickAngle(dev, STICK_LEFT),
		GamepadStickLength(dev, STICK_LEFT),
		rx, ry,
		GamepadStickAngle(dev, STICK_RIGHT),
		GamepadStickLength(dev, STICK_RIGHT),
		GamepadTriggerLength(dev, TRIGGER_LEFT),
		GamepadTriggerLength(dev, TRIGGER_RIGHT));
	printw("U:%d D:%d L:%d R:%d ",
		GamepadButtonDown(dev, BUTTON_DPAD_UP),
		GamepadButtonDown(dev, BUTTON_DPAD_DOWN),
		GamepadButtonDown(dev, BUTTON_DPAD_LEFT),
		GamepadButtonDown(dev, BUTTON_DPAD_RIGHT));
	printw("A:%d B:%d X:%d Y:%d Bk:%d St:%d ",
		GamepadButtonDown(dev, BUTTON_A),
		GamepadButtonDown(dev, BUTTON_B),
		GamepadButtonDown(dev, BUTTON_X),
		GamepadButtonDown(dev, BUTTON_Y),
		GamepadButtonDown(dev, BUTTON_BACK),
		GamepadButtonDown(dev, BUTTON_START));
	printw("LB:%d RB:%d LS:%d RS:%d\n",
		GamepadButtonDown(dev, BUTTON_LEFT_SHOULDER),
		GamepadButtonDown(dev, BUTTON_RIGHT_SHOULDER),
		GamepadButtonDown(dev, BUTTON_LEFT_THUMB),
		GamepadButtonDown(dev, BUTTON_RIGHT_THUMB));
}
uint8_t tchar = 0xFF;

bool flag_auto = false;
float c1y = 0.6;
float c1z = atan2(270,200);

void sendOld(){
	for(int k = 0; k<24; k++){
		old_data[k] = data[k];
	}
}
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

int flag_get_ball = 0;
bool flag_orient = false;
bool flag_sweep = false;
int dest = 0;
int r = 0;
int x = 0;
int y = 0;
// bool flag_time = false;
int dec_buf = 5;

int deci[5];
int solid = 0;
int max_index = 0;
int min_index = 1;


void autoGreen(const data_massage::ETHER& msg3){
	green_u = msg3.above;
	green_l = msg3.left;
	green_r = msg3.right;
	green_e = msg3.exist;
	green_l2 = msg3.left2;
	green_r2 = msg3.right2;
}

void autoOrient(const data_massage::QTUM& msg2){
	yaw_now = msg2.data;
}

void autoBallTrack(const data_massage::BTC& msg)
{
	// dataInit();
	if(msg.real){
		if(!Blue_left){
			Blue_left = true;
		}
	}

	// outfile << "||deci: <";
	for(int k = 0; k<dec_buf-1; k++){

		deci[k] = deci[k+1];
		// outfile<<deci[k]<<"><";
	}
	deci[dec_buf-1] = msg.data2;

	// outfile << deci[dec_buf-1]<<">"<<endl;

	dest = 0;
	solid = 0;
	for(int k = 0 ; k < dec_buf; k++){

		dest = dest + deci[k];
		solid++;
	}
	if(solid != dec_buf){
		printf("\nERROR####################\n");
	}

	r = msg.real;
	x = msg.data;
	y = msg.data2;
	int dx;
	float v_x, v_z, v_y, p_l;

	// outfile <<"||r: " <<r<<"||x: "<<x<<"||y: "<<y<<endl;

	if(x==-1)
	{
		// dataInit();
		data[8] = 0.5;
		// write(c_socket, data, sizeof(data));
		// ros::Duration(0.1).sleep();
		return;
	}
	if(x==0){
		return;
	}
	//
	// if(r==0)
	// {
	// 	// dataInit();
	// 	// write(c_socket, data, sizeof(data));
	// 	// ros::Duration(0.1).sleep();
	// 	data[2] = old_data[2];
	// 	data[3] = old_data[3];
	// 	printf("line269line269line269line269line269line269line269line269");
	// 	return;
	// }
	// dataInit();



	//
	// data[2] = M_PI/2.0;
	// data[3] = 0.1;

	// if(x<(resol_x/2 - RANGE_DEAD2)){
	// 	//move right
	// 	data[0] =  strafe_max *(-1.0);
	// 	//trun right
	// 	data[4] = turn_max*((x-100.0)/100);
	//
	// 	// data[2] = 0;
	// 	// data[3] = data[0];
	// }else
	if(y<200){

		if(x<(resol_x/2 - RANGE_DEAD1)){
			//move right
			data[4] = -1*turn_min +turn_max /(resol_x/2 - RANGE_DEAD1)* (x-(resol_x/2 - RANGE_DEAD1));

		}else if(x<(resol_x/2 + RANGE_DEAD1)){
			//nothing
			data[2] = M_PI/2.0;
			data[3] = (450-y)/(450.0)*(strafe_max-strafe_min)+strafe_min;
			data[4] = 0;

		}else if(x<=resol_x){
			//move left
			// data[0] = strafe_max * (1.0);
			//turn left
			data[4] = turn_min + turn_max /(resol_x/2 - RANGE_DEAD1)* (x-(resol_x/2 + RANGE_DEAD1));

		}
	}
	else{

		if(x<(resol_x/2 - RANGE_DEAD2)){
			//move right
			data[4] = -1*turn_min +turn_max /(resol_x/2 - RANGE_DEAD2)* (x-(resol_x/2 - RANGE_DEAD2));

		}else if(x<(resol_x/2 + RANGE_DEAD2)){
			//nothing
			data[2] = M_PI/2.0;
			data[3] = (450-y)/(450.0)*(strafe_max-strafe_min)+strafe_min;
			data[4] = 0;
			if((dest/solid)>445){
				flag_sweep = true;
				printf("low ball detected\n");
				// printf("(%d,%d,)",r,dest);
			}else
				flag_sweep = false;


		}else if(x<=resol_x){
			//move left
			// data[0] = strafe_max * (1.0);
			//turn left
			data[4] = turn_min + turn_max /(resol_x/2 - RANGE_DEAD2)* (x-(resol_x/2 + RANGE_DEAD2));

		}
	}
}

int main(int argc, char **argv) {



	ros::init(argc, argv, "XboxCtrl");
    ros::start();

	int secs = ros::Time::now().toSec();
	secs = secs%10000;
	ostringstream strname;
	strname << secs;

	string fileName = strname.str();

	fileName += ".txt";
	outfile.open(fileName.c_str(), ios::app);
	// ros::NodeHandle nh;
	// ros::Subscriber sub = nh.subscribe("/positions", 1, autoBallTrack);
	memset(deci,0,dec_buf*4);

	int ch, i, j, k;
	float lx, ly, rx, ry;
	initscr();
	cbreak();
	noecho();
	timeout(1);

	GamepadInit();

	c_socket = socket(PF_INET, SOCK_STREAM, 0);
	printw("socket created\n");
	c_addr.sin_addr.s_addr = inet_addr(IPADDR);
	c_addr.sin_family = AF_INET;
	c_addr.sin_port = htons(PORT);

	if(connect(c_socket, (struct sockaddr*) &c_addr, sizeof(c_addr)) == -1){
		printf("Failed to connect\n");
		close(c_socket);
		return -1;
	}

		ros::NodeHandle nh;
		ros::Subscriber sub2 = nh.subscribe("yaw",1, autoOrient);
		ros::Subscriber sub = nh.subscribe("BTCGAZUA", 1, autoBallTrack);

		ros::Subscriber sub3 = nh.subscribe("ETHERGAZUA",1, autoGreen);

	while ((ch = getch()) != 'q') {
		GamepadUpdate();

		double dev = GAMEPAD_0;
		GAMEPAD_DEVICE _dev = static_cast<GAMEPAD_DEVICE>(dev);
		GamepadStickNormXY(_dev, STICK_LEFT, &lx, &ly);
		GamepadStickNormXY(_dev, STICK_RIGHT, &rx, &ry);

		// data[2] = GamepadStickAngle(_dev, STICK_LEFT);
		fake = GamepadStickLength(_dev, STICK_LEFT);
		// data[3] = GamepadStickLength(_dev, STICK_LEFT);
		// data[6] = GamepadStickAngle(_dev, STICK_RIGHT);
		// data[7] = GamepadStickLength(_dev, STICK_RIGHT);
		// data[10] = GamepadButtonDown(_dev, BUTTON_DPAD_UP);
		// data[11] = GamepadButtonDown(_dev, BUTTON_DPAD_DOWN);
		// data[12] = GamepadButtonDown(_dev, BUTTON_DPAD_LEFT);
		// data[13] = GamepadButtonDown(_dev, BUTTON_DPAD_RIGHT);
		data[14] = GamepadButtonDown(_dev, BUTTON_A);
		data[15] = GamepadButtonDown(_dev, BUTTON_B);
		data[16] = GamepadButtonDown(_dev, BUTTON_X);
		data[17] = GamepadButtonDown(_dev, BUTTON_Y);
		// data[18] = GamepadButtonDown(_dev, BUTTON_BACK);
		data[19] = GamepadButtonDown(_dev, BUTTON_START);
		// data[20] = GamepadButtonDown(_dev, BUTTON_LEFT_SHOULDER);
		// data[21] = GamepadButtonDown(_dev, BUTTON_RIGHT_SHOULDER);
		// data[22] = GamepadButtonDown(_dev, BUTTON_LEFT_THUMB);
		// data[23] = GamepadButtonDown(_dev, BUTTON_RIGHT_THUMB);

		outfile << "";
		// Auto Moving
		// if(data[19] > 0.9)
		// 	flag_auto = !flag_auto;



		if(data[14] > 0.5){
			flag_orient = false;
			gohome = false;
			flag_auto = true;
			flag_final = false;
		}

		if(data[16] > 0.5){
			flag_orient = true;
			gohome = false;
			flag_auto = false;
			flag_final = false;
		}
		if(data[17] > 0.5){
			flag_orient = false;
			gohome = false;
			flag_auto = false;
			flag_final = false;
		}

		yaw_temp = yaw_now-yaw_init;
		if(yaw_temp > M_PI){
			yaw_temp = yaw_temp - 2*M_PI;
		}
		if(yaw_temp < -1*M_PI){
			yaw_temp = yaw_temp + 2*M_PI;
		}

		if(flag_final){

			FinalOri();
			if(yaw_temp > M_PI - 0.03 || yaw_temp < -1.0*M_PI + 0.03){
				data[2] = -1* M_PI/2.0;
				data[3] = 0.15;
				data[4] = 0;
				for(int j = 0 ; j<50; j++){
					write(c_socket, data, sizeof(data));
					ros::Duration(0.05).sleep();
				}
				dataInit();

				data[9] = 1;
				for(int j = 0; j<50; j++){
					// printf("son");
					write(c_socket, data, sizeof(data));
					ros::Duration(0.05).sleep();
				}
				flag_orient = false;
				gohome = false;
				flag_auto = false;
				flag_final = false;
			}

		}
		if(gohome){
			dataInit();

			flag_final1 = flag_final2;
			flag_final2 = flag_final3;
			if(yaw_temp > 0.15 || yaw_temp < -0.15){
				flag_error =true;
			}
			if(flag_error){
				Phase2Ori();
			}else{
				Phase2();
			}
			if(flag_final1 && flag_final2 && flag_final3){
				flag_final = true;
			}else{
				flag_final = false;
			}
			write(c_socket,data,sizeof(data));
			ros::Duration(0.05).sleep();
			ros::spinOnce();
			continue;
		}


		if(flag_orient){
			dataInit();
			if(green_e){
				for(int j = 0; j<20; j++){
					write(c_socket, data, sizeof(data));
					ros::Duration(0.05).sleep();
				}


				if(yaw_temp >0 ){
					Idle_right = false;
				}else{
					Idle_right = true;
				}
				gohome = true;
				flag_orient = false;
			}else{
				data[4] = 0.2;
				write(c_socket, data, sizeof(data));
				ros::Duration(0.05).sleep();

			}
			ros::spinOnce();
			continue;

		}





		if(data[15] >0.5){
			outfile << "~~~~~~~~~~~~CALIBRATING~~~~~~~~~~~~~~"<<endl;
			outfile << "N: " <<n << endl;
			outfile << "current yaw: "<<yaw_now <<endl;
			Calib();
			printf("calib\n");
			outfile << "CAL_AVG yaw: "<<yaw_init <<endl;
			outfile << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<endl;
		}
		else{
			n = 0;
		}

		outfile << "####yaw: "<< yaw_now << endl;

		if(flag_orient){
			outfile << "@@@@@orientation mode@@@@@"<<endl;
			Orient();
			dataInit();
			// flag_orient = false;
			ros::spinOnce();
			continue;
		}
		if(!flag_auto)
		{

			dataInit();
			write(c_socket, data, sizeof(data));
			ros::Duration(0.05).sleep();
			ros::spinOnce();

		}
		else
		{
			// printf("(r,dest,detct) = %d,%d,%d",r,dest,flag_detect);

			// if(fake>data[3]){
			// 	data[2] =M_PI/2.0;
			// }
			// data[3] = max(data[3],fake);
			// data[8] = max(data[8], GamepadTriggerLength(_dev, TRIGGER_LEFT))


			sendOld();


			if(dest/dec_buf ==0){
				if(Blue_left){
					Idle_right = false;
				}
				Blue_left = false;
				dataInit();
				outfile<<"|||||||||noball||||||||"<<endl;
				printf("noball!!!!!!!!!!!!!!!!!!");
				if(Idle_right){
					data[4] = turn_max;
				}else{
					data[4] = -turn_max;
				}
				yaw_temp = yaw_now - yaw_old;
				if(yaw_temp > M_PI){
					yaw_temp = yaw_temp - 2*M_PI;
				}
				if(yaw_temp < -1*M_PI){
					yaw_temp = yaw_temp + 2*M_PI;
				}

				no_ball = no_ball + yaw_temp;

				if(no_ball > 2.0*M_PI || no_ball < -2.0*M_PI){
					flag_orient = true;
					gohome = false;
					flag_auto = false;
					flag_final = false;
					no_ball = 0.0;
				}

			}else{
				no_ball = 0.0;
				if(!Idle_right)
					Idle_right = true;
			}
			yaw_old = yaw_now;

			if(flag_sweep){
				data[2] = M_PI/2.0;
				data[3] = 0.2;
				data[4] = 0;

				for(int j=0; j<40; j++){
					// printf("damn");
					write(c_socket, data, sizeof(data));
					ros::Duration(0.05).sleep();
				}
				dataInit();
				data[2] = M_PI/2.0;
				data[3] = 0.15;
				data[8] = 1;
				for(int j = 0; j<50; j++){
					// printf("son");
					write(c_socket, data, sizeof(data));
					ros::Duration(0.05).sleep();
				}
				// printf("\n");

				flag_sweep = false;

				dataInit();
				memset(deci,0,dec_buf*4);
			}
			// printf("%d",flag_detect);
			outfile<<"||||| ";
			for(int j=0 ; j<6; j++){
				outfile<<data[j]<<"  ";
			}
			outfile<<endl;
			write(c_socket, data, sizeof(data));

			ros::Duration(0.05).sleep();

			// dest = 0;
			ros::spinOnce();
		}

		if (ch == 'r') {
			for (i = 0; i != GAMEPAD_COUNT; ++i) {
				GamepadSetRumble(static_cast<GAMEPAD_DEVICE>(i), 0.25f, 0.25f);
			}
		}
		// printf("{About to send: %.2f,%.2f,%.2f,%.2f,%.2f}//", data[2],data[3],data[4],data[5],data[8]);
		update(GAMEPAD_0);
		//update(GAMEPAD_1);
		//update(GAMEPAD_2);
		//update(GAMEPAD_3);

		//for (i = 0; i != GAMEPAD_COUNT; ++i)
		{
			GAMEPAD_DEVICE _i = static_cast<GAMEPAD_DEVICE>(0);
			if (GamepadIsConnected(_i)) {
				for (j = 0; j != BUTTON_COUNT; ++j) {
					GAMEPAD_BUTTON _j = static_cast<GAMEPAD_BUTTON>(j);
					if (GamepadButtonTriggered(_i, _j)) {
						logevent("[%d] button triggered: %s", i, button_names[j]);
					} else if (GamepadButtonReleased(_i, _j)) {
						logevent("[%d] button released:  %s", i, button_names[j]);
					}
				}
				for (j = 0; j != TRIGGER_COUNT; ++j) {
					GAMEPAD_TRIGGER _j = static_cast<GAMEPAD_TRIGGER>(j);
					if (GamepadTriggerTriggered(_i, _j)) {
						logevent("[%d] trigger pressed:  %d", i, j);
					} else if (GamepadTriggerReleased(_i, _j)) {
						logevent("[%d] trigger released: %d", i, j);
					}
				}
				for (j = 0; j != STICK_COUNT; ++j) {
					GAMEPAD_STICK _j = static_cast<GAMEPAD_STICK>(j);
					for (k = 0; k != STICKDIR_COUNT; ++k) {
						GAMEPAD_STICKDIR _k = static_cast<GAMEPAD_STICKDIR>(k);
						if (GamepadStickDirTriggered(_i, _j, _k)) {
							logevent("[%d] stick direction:  %d -> %d", i, j, k);
						}
					}
				}
			}
		}

		move(6, 0);
		printw("(q)uit (r)umble");

		refresh();
	}
	outfile.close();
	close(c_socket);

	ros::shutdown();

	endwin();

	return 0;
}
