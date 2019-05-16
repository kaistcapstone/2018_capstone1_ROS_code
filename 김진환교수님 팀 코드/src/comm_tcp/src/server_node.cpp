/*************************************************************************
 * Author: Abhinav Jain
 * Contact: abhinavjain241@gmail.com, abhinav.jain@heig-vd.ch
 * Date: 28/06/2016
 *
 * This file contains source code to the server node of the ROS package
 * comm_tcp developed at LaRA (Laboratory of Robotics and Automation)
 * as part of my project during an internship from May 2016 - July 2016.
 *
 * (C) All rights reserved. LaRA, HEIG-VD, 2016 (http://lara.populus.ch/)
 ***************************************************************************/
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include "std_msgs/String.h"

using namespace std;

void error(const char *msg) {
    perror(msg);
    exit(1);
}

int main (int argc, char** argv)
{
  int iter = 0;

  ros::init(argc, argv, "server_node");
  ros::NodeHandle nh;
  ros::Publisher server_pub = nh.advertise<std_msgs::String>("/realsense/biggest_balls", 1000);

  // ros::Publisher server_pub = nh.advertise<std_msgs::String>("/realsense/biggest_ball", 1000);
  //Testing package is working fine
  int sockfd, newsockfd, portno; //Socket file descriptors and port number
  socklen_t clilen; //object clilen of type socklen_t
  char buffer[256]; //buffer array of size 256
  float buffer_float[24];
  struct sockaddr_in serv_addr, cli_addr; ///two objects to store client and server address
  std_msgs::String message;
  std::stringstream ss;
  int n;
  ros::Duration d(0.01); // 0.01 -> 100Hz

  //rosrun comm_tcp server_node [PORT] format is assumed
  if (argc < 2) {
    fprintf(stderr,"ERROR, no port provided\n");
    exit(1);
  }
  portno = atoi(argv[1]);
  cout << "Hello there! This node is listening on port " << portno << " for incoming connections" << endl;


  //socket fd declared
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0)
      error("ERROR opening socket");
  int enable = 1;
  if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0)
      error("setsockopt(SO_REUSEADDR) failed");

  //server initialization
  bzero((char *) &serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = INADDR_ANY;
  serv_addr.sin_port = htons(portno);

  //connecting socket
  if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
      error("ERROR on binding");

  //kernal initialization
  if (listen(sockfd,5) == -1)
      error("ERROR on listening\n");

  // listen(sockfd,5);

  //make connecting socket (newsockfd) based on listening socket (sockfd)
  clilen = sizeof(cli_addr);
  newsockfd = accept(sockfd,
              (struct sockaddr *) &cli_addr,
              &clilen);
  if (newsockfd < 0)
       error("ERROR on accept");
  while(ros::ok()) {
      // bzero(buffer_float,sizeof(buffer_float));
      // if ((n=read(newsockfd,buffer_float, sizeof(buffer_float)))<0){
      //   error("ERROR reading from socket");
      // }
      //
      //
      // for(int i = 0; i<n; i++)
      //   printf("%f",buffer_float[i]);
      //
      // printf("\n");
      //
      // printf("%d th iteration\n", iter);
      // iter++;


      // newsockfd = accept(sockfd,
      //             (struct sockaddr *) &cli_addr,
      //             &clilen);
      // if (newsockfd < 0)
      //      error("ERROR on accept");

      ss.str(std::string()); //Clear contents of string stream
      bzero(buffer,256);
      n = read(newsockfd,buffer,sizeof(buffer_float));

      if (n < 0) error("ERROR reading from socket");

      printf("Here is the message: %s\n",buffer);
      message.data = ss.str();
      ROS_INFO("%s", message.data.c_str());

      memcpy(buffer_float, buffer, sizeof(buffer));

      for (int i = 0; i < 5; i++){
      printf("%.3f, ",  (float)buffer_float[i]);
      }
      printf("%.3f", (float)buffer_float[8]);


      printf("\n");
      server_pub.publish(message);
      n = write(newsockfd,"I got your message",18);
      if (n < 0) error("ERROR writing to socket");
      // close(newsockfd);
      // close(sockfd);
      ros::spinOnce();
      d.sleep();
  }

  return 0;
}
