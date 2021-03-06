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


extern "C" {
	#include "xbox_ctrl/gamepad.h"
}

using namespace std;

#define PORT 4000
#define IPADDR "172.16.0.1"
//#define IPADDR "127.0.0.1"
#define resol_x 640
#define resol_y 480
#define strafe_max 0.7
#define strafe_min 0.3
#define turn_max 0.3

#define RANGE_DEAD1 40
#define RANGE_DEAD2 220
#define RANGE_STATE_CHANGE 300


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

int c_socket, s_socket;
struct sockaddr_in c_addr;
int len;
int n;
float data[24];
float old_data[24];
float fake = 0.0;
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

bool flag_auto = true;
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
	data[8] = 1; //GamepadTriggerLength(_dev, TRIGGER_LEFT);
	data[9] = 0; //GamepadTriggerLength(_dev, TRIGGER_RIGHT);
	data[10] = 0; //GamepadButtonDown(_dev, BUTTON_DPAD_UP);
	data[11] = 0; //GamepadButtonDown(_dev, BUTTON_DPAD_DOWN);
	data[12] = 0; //GamepadButtonDown(_dev, BUTTON_DPAD_LEFT);
	data[13] = 0; //GamepadButtonDown(_dev, BUTTON_DPAD_RIGHT);
	//data[14] = 0; //GamepadButtonDown(_dev, BUTTON_A); // duct on/off
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

bool flag_detect = false;
bool flag_sweep = false;
int dest = 0;
int r = 0;
int x = 0;
int y = 0;
int solid= 4;
// bool flag_time = false;
int dec_buf = 6;

int deci[6];

// *deci = 4;
// int deci[dec_buf];
int max_index = 0;
int min_index = 1;
// void autoBallTrack(const core_msgs::ball_position& msg)
void autoBallTrack(const data_massage::BTC& msg)
{
	dataInit();

	for(int k = 0; k<dec_buf-1; k++){
		deci[k] = deci[k+1];
	}
	deci[dec_buf-1] = msg.data2;
	min_index = (min_element(deci, deci+dec_buf)-deci);
	max_index = (max_element(deci, deci+dec_buf)-deci);

	if(min_index == max_index){
		if(min_index ==0){
			max_index = 1;
		}else{
			max_index = 0;
		}
	}
	dest = 0;
	solid = 0;
	for(int k = 0 ; k < dec_buf; k++){
		if(k == min_index){
			continue;
		}
		if(k == max_index){
			continue;
		}
		dest = dest + deci[k];
		solid++;
	}
	if(solid != dec_buf-2){
		printf("\nERROR####################\n");
	}


	// dest =1;
	// printf("catched\n");
	r = msg.real;
	x = msg.data;
	// int x = (int)msg.img_x[0];
	// int dy = (int)msg.img_y[0];
	y = msg.data2;
	int dx;
	float v_x, v_z, v_y, p_l;
	// dataInit();
	// if(flag_detect && y<=200){
	// 	printf("fuckyou\n");
	// 	flag_sweep = true;
	// }
	// else{
	// 	flag_detect = false;
	// }


	if(x==-1)
	{
		// dataInit();
		data[8] = 0.5;
		// write(c_socket, data, sizeof(data));
		// ros::Duration(0.1).sleep();
		return;
	}

	// flag_get_ball++;

	if(flag_get_ball >= 5)
	{
		// dataInit();
		data[14] = 0;
		// write(c_socket, data, sizeof(data));
		flag_get_ball = 0;
		// ros::Duration(0.1).sleep();
		return;
	}
	if(x==0)
	{
		// dataInit();
		// write(c_socket, data, sizeof(data));
		// ros::Duration(0.1).sleep();
		data[2] = old_data[2];
		data[3] = old_data[3];
		return;
	}
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

	if(x<(resol_x/2 - RANGE_DEAD1)){
		//move right
		data[4] = turn_max /(resol_x/2 - RANGE_DEAD1)* (x-(resol_x/2 - RANGE_DEAD1));

	}else if(x<(resol_x/2 + RANGE_DEAD1)){
		//nothing
		data[2] = M_PI/2.0;
		data[3] = (450-y)/(450.0)*(strafe_max-strafe_min)+strafe_min;
		data[4] = 0;
		if((dest/solid)>450){
			flag_sweep = true;
			printf("low ball detected\n");
			// printf("(%d,%d,)",r,dest);
		}else
			flag_sweep = false;
	}else if(x<=resol_x){
		//move left
		// data[0] = strafe_max * (1.0);
		//turn left
		data[4] = turn_max /(resol_x/2 - RANGE_DEAD1)* (x-(resol_x/2 + RANGE_DEAD1));

	}
	// if(x<(resol_x/2 + RANGE_DEAD2)){
	// 	//move left
	// 	data[0] = strafe_max * ((x-340.0)/200);
	//
	// 	// data[2] = M_PI/2.0;
	// 	// data[3] = data[0] * (-1.0);
	// }else


	if(abs(data[3]-old_data[3])>(0.5*strafe_max)){
		data[3] = 0.5*data[3];
	}
	if(abs(data[4]-old_data[4])>(0.4)){
		data[4] = 0.5*data[4];
	}

	// write(c_socket, data, sizeof(data));
	// printf("msgcallback: %d\n", flag_sweep);


	// cout << "send1" << endl;
	// ros::Duration(0.1).sleep();
	// dataInit();
	// write(c_socket, data, sizeof(data));
	// cout << "send2" << endl;
	// ros::Duration(0.05).sleep();
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "XboxCtrl");
    ros::start();

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
		ros::Subscriber sub = nh.subscribe("BTCGAZUA", 1, autoBallTrack);


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
		// data[15] = GamepadButtonDown(_dev, BUTTON_B);
		// data[16] = GamepadButtonDown(_dev, BUTTON_X);
		// data[17] = GamepadButtonDown(_dev, BUTTON_Y);
		// data[18] = GamepadButtonDown(_dev, BUTTON_BACK);
		// data[19] = GamepadButtonDown(_dev, BUTTON_START);
		// data[20] = GamepadButtonDown(_dev, BUTTON_LEFT_SHOULDER);
		// data[21] = GamepadButtonDown(_dev, BUTTON_RIGHT_SHOULDER);
		// data[22] = GamepadButtonDown(_dev, BUTTON_LEFT_THUMB);
		// data[23] = GamepadButtonDown(_dev, BUTTON_RIGHT_THUMB);

		// Auto Moving
		if(data[19] > 0.9)
			flag_auto = !flag_auto;

		if(!flag_auto)
		{
			data[0] = lx*data[3];
			data[1] = ly*data[3];
			data[4] = rx*data[7];
			data[5] = ry*data[7];
			data[8] = GamepadTriggerLength(_dev, TRIGGER_LEFT);
			data[9] = GamepadTriggerLength(_dev, TRIGGER_RIGHT);
			data[14] = GamepadButtonDown(_dev, BUTTON_A); // duct on/off
			write(c_socket, data, sizeof(data));
			ros::Duration(0.01).sleep();
		}
		else
		{
			// printf("(r,dest,detct) = %d,%d,%d",r,dest,flag_detect);
			printf("About to send: %f,%f\n", data[2],data[3]);
			// if(fake>data[3]){
			// 	data[2] =M_PI/2.0;
			// }
			// data[3] = max(data[3],fake);
			// data[8] = max(data[8], GamepadTriggerLength(_dev, TRIGGER_LEFT))


			sendOld();


			if(dest/solid ==0){
				dataInit();
				data[4] = turn_max/2;
				// if(flag_detect){
				// 	printf("noW!\n");
				// 	flag_sweep = true;
				//
				// }
				// flag_detect = false;
			}


			if(flag_sweep){
				data[2] = M_PI/2.0;
				data[3] = 0.2;
				data[4] = 0;

				for(int j=0; j<55; j++){
					// printf("damn");
					write(c_socket, data, sizeof(data));
					ros::Duration(0.05).sleep();
				}
				dataInit();
				data[2] = M_PI/2.0;
				data[3] = 0.05;
				data[8] = 0;
				for(int j = 0; j<50; j++){
					// printf("son");
					write(c_socket, data, sizeof(data));
					ros::Duration(0.05).sleep();
				}
				// printf("\n");

				flag_sweep = false;
				flag_detect = false;

				dataInit();
				memset(deci,0,dec_buf*4);
			}
			// printf("%d",flag_detect);
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
	close(c_socket);

	ros::shutdown();

	endwin();

	return 0;
}
