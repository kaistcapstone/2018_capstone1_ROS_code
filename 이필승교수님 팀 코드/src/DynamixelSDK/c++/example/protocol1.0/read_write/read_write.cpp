/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: Ryu Woon Jung (Leon) */

//
// *********     Read and Write Example      *********
//
//
// Available DXL model on this example : All models using Protocol 1.0
// This example is tested with a DXL MX-28, and an USB2DYNAMIXEL
// Be sure that DXL MX properties are already set as %% ID : 1 / Baudnum : 34 (Baudrate : 57600)
//

#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>

#include "dynamixel_sdk.h"                                  // Uses Dynamixel SDK library

// Control table address
#define ADDR_MX_TORQUE_ENABLE           24                  // Control table address is different in Dynamixel model
#define ADDR_MX_GOAL_POSITION           30
#define ADDR_MX_PRESENT_POSITION        36

#define ADDR_MX_MOVING_SPEED 32
#define ADDR_MX_PRESENT_SPEED 38


// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID1                          1                   // Dynamixel ID: 1
#define DXL_ID2  2
#define DXL_ID3  3
#define DXL_ID4  4
#define BAUDRATE                        57600
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      0                 // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      3000                // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     10                  // Dynamixel moving status threshold

#define DXL_MINIMUM_SPEED_VALUE_CCW      0
#define DXL_MAXIMUM_SPEED_VALUE_CCW      1000
#define DXL_MINIMUM_SPEED_VALUE_CW      1024
#define DXL_MAXIMUM_SPEED_VALUE_CW      2000

#define ESC_ASCII_VALUE                 0x1b

int getch()
{
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
#elif defined(_WIN32) || defined(_WIN64)
  return _getch();
#endif
}

int kbhit(void)
{
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
#elif defined(_WIN32) || defined(_WIN64)
  return _kbhit();
#endif
}

int main()
{
  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  int index = 0;
  int dxl_comm_result1 = COMM_TX_FAIL;             // Communication result
  int dxl_comm_result2 = COMM_TX_FAIL;
  int dxl_comm_result3 = COMM_TX_FAIL;
  int dxl_comm_result4 = COMM_TX_FAIL;
  //int dxl_goal_position[2] = {DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE};         // Goal position
  int dxl_velocity_CW[2] = {DXL_MINIMUM_SPEED_VALUE_CW, DXL_MAXIMUM_SPEED_VALUE_CW};
  int dxl_velocity_CCW[2] = {DXL_MINIMUM_SPEED_VALUE_CCW, DXL_MAXIMUM_SPEED_VALUE_CCW};

  uint8_t dxl_error = 0;                          // Dynamixel error
  uint16_t dxl_present_velocity = 0;              // Present position

  // Open port
  if (portHandler->openPort())
  {
    printf("Succeeded to open the port!\n");
  }
  else
  {
    printf("Failed to open the port!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
    printf("Succeeded to change the baudrate!\n");
  }
  else
  {
    printf("Failed to change the baudrate!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // Enable Dynamixel Torque
  dxl_comm_result1 = packetHandler->write1ByteTxRx(portHandler, DXL_ID1, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  dxl_comm_result2 = packetHandler->write1ByteTxRx(portHandler, DXL_ID2, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  dxl_comm_result3 = packetHandler->write1ByteTxRx(portHandler, DXL_ID3, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  dxl_comm_result4 = packetHandler->write1ByteTxRx(portHandler, DXL_ID4, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result1 != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result1));
  }
  else if (dxl_error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }
  else
  {
    printf("Dynamixel has been successfully connected \n");
  }

  while(1)
  {
    printf("Press any key to continue! (or press ESC to quit!)\n");
    if (getch() == ESC_ASCII_VALUE)
      break;

    // Write goal position
    dxl_comm_result1 = packetHandler->write2ByteTxRx(portHandler, DXL_ID1, ADDR_MX_MOVING_SPEED, dxl_velocity_CW[index], &dxl_error);
    dxl_comm_result2 = packetHandler->write2ByteTxRx(portHandler, DXL_ID2, ADDR_MX_MOVING_SPEED, dxl_velocity_CCW[index], &dxl_error);
    dxl_comm_result3 = packetHandler->write2ByteTxRx(portHandler, DXL_ID3, ADDR_MX_MOVING_SPEED, dxl_velocity_CCW[index], &dxl_error);
    dxl_comm_result4 = packetHandler->write2ByteTxRx(portHandler, DXL_ID4, ADDR_MX_MOVING_SPEED, dxl_velocity_CW[index], &dxl_error);
    if (dxl_comm_result1 != COMM_SUCCESS)
    {
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result1));
    }
    else if (dxl_error != 0)
    {
      printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }

    do
    {
      // Read present position
      dxl_comm_result1 = packetHandler->read2ByteTxRx(portHandler, DXL_ID1, ADDR_MX_PRESENT_SPEED, &dxl_present_velocity, &dxl_error);
      dxl_comm_result2 = packetHandler->read2ByteTxRx(portHandler, DXL_ID2, ADDR_MX_PRESENT_SPEED, &dxl_present_velocity, &dxl_error);
      dxl_comm_result3 = packetHandler->read2ByteTxRx(portHandler, DXL_ID3, ADDR_MX_PRESENT_SPEED, &dxl_present_velocity, &dxl_error);
      dxl_comm_result4 = packetHandler->read2ByteTxRx(portHandler, DXL_ID4, ADDR_MX_PRESENT_SPEED, &dxl_present_velocity, &dxl_error);
      if (dxl_comm_result1 != COMM_SUCCESS)
      {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result1));
      }
      else if (dxl_error != 0)
      {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
      }

      printf("[ID:%03d] MovingSpeed:%03d  PresVel:%03d\n", DXL_ID1, dxl_velocity_CW[index], dxl_present_velocity);
      printf("[ID:%03d] MovingSpeed:%03d  PresVel:%03d\n", DXL_ID2, dxl_velocity_CCW[index], dxl_present_velocity);
      printf("[ID:%03d] MovingSpeed:%03d  PresVel:%03d\n", DXL_ID3, dxl_velocity_CCW[index], dxl_present_velocity);
      printf("[ID:%03d] MovingSpeed:%03d  PresVel:%03d\n", DXL_ID4, dxl_velocity_CW[index], dxl_present_velocity);

    }
    while(dxl_present_velocity > 10 || dxl_present_velocity > 1034);

    // Change goal position
    if (index == 0)
    {
      index = 1;
    }
    else
    {
      index = 0;
    }
  }

  // Disable Dynamixel Torque
  dxl_comm_result1 = packetHandler->write1ByteTxRx(portHandler, DXL_ID1, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  dxl_comm_result2 = packetHandler->write1ByteTxRx(portHandler, DXL_ID2, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  dxl_comm_result3 = packetHandler->write1ByteTxRx(portHandler, DXL_ID3, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  dxl_comm_result4 = packetHandler->write1ByteTxRx(portHandler, DXL_ID4, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result1 != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result1));

  }
  else if (dxl_error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }

  // Close port
  portHandler->closePort();

  return 0;
}
