#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "core_msgs/ball_position.h"
#include "opencv2/opencv.hpp"
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>

#include "opencv2/imgproc.hpp"
#include <iostream>
#include <string>
#include <stdlib.h>
#include <math.h>

using namespace cv;
using namespace std;

void on_low_h_thresh_trackbar_green(int, void *);
void on_high_h_thresh_trackbar_green(int, void *);
void on_low_s_thresh_trackbar_green(int, void *);
void on_high_s_thresh_trackbar_green(int, void *);
void on_low_v_thresh_trackbar_green(int, void *);
void on_high_v_thresh_trackbar_green(int, void *);


int low_h_g=25, low_s_g=100, low_v_g=100;
int high_h_g=75, high_s_g=255, high_v_g=255;



// // Declaration of trackbar functions to set HSV colorspace's parameters (red) red is divided into 2 regions
// void on_low_h_thresh_trackbar_red(int, void *);
// void on_high_h_thresh_trackbar_red(int, void *);
// void on_low_h2_thresh_trackbar_red(int, void *);
// void on_high_h2_thresh_trackbar_red(int, void *);
// void on_low_s_thresh_trackbar_red(int, void *);
// void on_high_s_thresh_trackbar_red(int, void *);
// void on_low_v_thresh_trackbar_red(int, void *);
// void on_high_v_thresh_trackbar_red(int, void *);
//
// //Initial parameter settings for the trackbar setting (red) red is divided into 2 reginos so there is h2
// int low_h2_r=160, high_h2_r=180;
// int low_h_r=0, low_s_r=100, low_v_r=136;
// int high_h_r=10, high_s_r=255, high_v_r=255;

//Declaration of trackbar functions for color blue
void on_low_h_thresh_trackbar_blue(int, void *);
void on_high_h_thresh_trackbar_blue(int, void *);
void on_low_s_thresh_trackbar_blue(int, void *);
void on_high_s_thresh_trackbar_blue(int, void *);
void on_low_v_thresh_trackbar_blue(int, void *);
void on_high_v_thresh_trackbar_blue(int, void *);

//Initial parameterr settings for the trackber setting (blue)
int low_h_b=100, low_s_b=100, low_v_b=100;
int high_h_b=140, high_s_b=255, high_v_b=255;

// Declaration of functions that changes data types
string intToString(int n);// Declaration of functions that changes int data to String
string floatToString(float f); //change float to string

//for image smoothing (refer to Open CV)
void morphOps(Mat &thresh);


// Declaration of functions that calculates the ball position from pixel position
vector<float> pixel2point(Point center, int radius);//Px,Py screen coordinate to Camera coordinate Xc, Yc, Zc


// Declaration of trackbars function that set canny edge's parameters (for red ball)
// void on_canny_edge_trackbar_red(int, void *);
// int lowThreshold_r = 100;
// int ratio_r = 3;
// int kernel_size_r = 3;

// Declaration of trackbars function that set canny edge's parameters (for blue ball)
void on_canny_edge_trackbar_blue(int, void *);
int lowThreshold_b = 100;
int ratio_b = 3;
int kernel_size_b = 3;

void on_canny_edge_trackbar_green(int, void *);
int lowThreshold_g = 100;
int ratio_g = 3;
int kernel_size_g = 3;



// Initialization of variable for dimension of the target
float fball_radius = 0.062 ; // meter


// Initialization of variable for camera calibration paramters (requires camera's specific parameters)
Mat distCoeffs;
float intrinsic_data[9] = {596.497705, 0, 318.023172, 0, 565.010517,271.337191, 0, 0, 1};
float distortion_data[5] = {0.073823, -0.079246, 0.005644, -0.005499,0};

// Initialization of variable for text drawing
double fontScale = 2;
int thickness = 3;
String text ;

//unit is in pixels The number should be considered with actual scale
int iMin_tracking_ball_size = 10;

//Initialization




Mat buffer(160,120,CV_8UC1);
ros::Publisher pub;
ros::Publisher pub_markers;

void ball_detect(){
  Mat frame, bgr_frame, hsv_frame,
  // hsv_frame_red, hsv_frame_red1,
  // hsv_frame_red_blur, hsv_frame_red2,hsv_frame_red_canny,
  hsv_frame_blue,  hsv_frame_blue_blur,  hsv_frame_blue_canny,
  hsv_frame_green, hsv_frame_green_blur, hsv_frame_green_canny, result;

    //create matrices for parameters for camera calibration
    Mat calibrated_frame;
    Mat intrinsic = Mat(3,3, CV_32FC1);
    Mat distCoeffs;

    //camera calibration cannot read float, it only reads MAT
    intrinsic = Mat(3, 3, CV_32F, intrinsic_data);
    distCoeffs = Mat(1, 5, CV_32F, distortion_data);

    //vec4i is vector with 4 dimensions, first two parameters are line's start point (x1,y1)
    //second two parameters are  line's end point (x2,y2)
    //this will set the region for contour
    // vector<Vec4i> hierarchy_r;
    vector<Vec4i> hierarchy_b;
    vector<Vec4i> hierarchy_g;

    //stores contours information in vector format
    // vector<vector<Point> > contours_r;
    vector<vector<Point> > contours_b;
    vector<vector<Point>> contours_g;

     Mat edges;  //assign a memory to save the edge images
//     Mat frame;  //assign a memory to save the images

     if(buffer.size().width!=640){ //if the size of the image is 320x240, then resized it to 640x480
         cv::resize(buffer, frame, cv::Size(640,480));
     }
     else{
         frame = buffer;
     }

     undistort(frame, calibrated_frame, intrinsic, distCoeffs);
     result = calibrated_frame.clone();


     medianBlur(calibrated_frame, calibrated_frame, 3);

     //convert colorspace from BGR to HSV
     cvtColor(calibrated_frame, hsv_frame, cv::COLOR_BGR2HSV);

     // Detect the object based on RGB and HSV Range Values
     // inRange(hsv_frame,Scalar(low_h_r,low_s_r,low_v_r),Scalar(high_h_r,high_s_r,high_v_r),hsv_frame_red1);
     // inRange(hsv_frame,Scalar(low_h2_r,low_s_r,low_v_r),Scalar(high_h2_r,high_s_r,high_v_r),hsv_frame_red2);
     inRange(hsv_frame,Scalar(low_h_b,low_s_b,low_v_b),Scalar(high_h_b,high_s_b,high_v_b),hsv_frame_blue);
     //red has to be divided into 2 parts, because of the HSV format
     inRange(hsv_frame,Scalar(low_h_g,low_s_g,low_v_g),Scalar(high_h_g,high_s_g,high_v_g),hsv_frame_green);

     //Gaussian weighting, for smoothing
     // addWeighted(hsv_frame_red1, 1.0, hsv_frame_red2, 1.0, 0.0,hsv_frame_red);


     //Additional Image smoothing process for both red and blue
     // morphOps(hsv_frame_red);
     morphOps(hsv_frame_blue);
     morphOps(hsv_frame_green);
     //Apply Gaussian blurs for both red and blue
     // GaussianBlur(hsv_frame_red, hsv_frame_red_blur, cv::Size(9, 9),2, 2);
     GaussianBlur(hsv_frame_blue, hsv_frame_blue_blur, cv::Size(9,9), 2, 2);
     GaussianBlur(hsv_frame_green, hsv_frame_green_blur, cv::Size(9,9), 2, 2);
     //apply canny edge filter for both red and blue
     // Canny(hsv_frame_red_blur, hsv_frame_red_canny, lowThreshold_r,lowThreshold_r*ratio_r, kernel_size_r);
     Canny(hsv_frame_blue_blur, hsv_frame_blue_canny, lowThreshold_b,lowThreshold_b*ratio_b, kernel_size_b);
     Canny(hsv_frame_green_blur, hsv_frame_green_canny, lowThreshold_g,lowThreshold_g*ratio_g, kernel_size_g);

     //apply contours for the red and blue colors with parameters set previously
     // findContours(hsv_frame_red_canny, contours_r, hierarchy_r,RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0, 0));
     findContours(hsv_frame_blue_canny, contours_b, hierarchy_b,RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0, 0));
     findContours(hsv_frame_green_canny, contours_g, hierarchy_g,RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0, 0));

     //store polygon contour informations in vecgtor format for countours with at least some specific size
     // vector<vector<Point> > contours_r_poly( contours_r.size() );
     vector<vector<Point> > contours_b_poly( contours_b.size() );
     vector<vector<Point> > contours_g_poly( contours_g.size() );
     //store center information of red and blue balls in vector format
     // vector<Point2f>center_r( contours_r.size() );
     vector<Point2f>center_b( contours_b.size() );
     vector<Point2f>center_g( contours_g.size() );
     //store radius information of red and blue balls in float values in vectors
     // vector<float>radius_r( contours_r.size() );
     vector<float>radius_b( contours_b.size() );
     vector<float>radius_g( contours_g.size() );

     core_msgs::ball_position msg;  //create a message for ball positions
     msg.size =contours_b.size(); //adjust the size of message. (*the size of message is varying depending on how many circles are detected)
     msg.img_x.resize(contours_b.size());  //adjust the size of array
     msg.img_y.resize(contours_b.size());  //adjust the size of array

     msg.img_r.resize(contours_b.size());
  //   msg.img_z.resize(contours_r.size());

	visualization_msgs::Marker ball_list;  //declare marker
	ball_list.header.frame_id = "/camera_link";  //set the frame
	ball_list.header.stamp = ros::Time::now();   //set the header. without it, the publisher may not publish.
	ball_list.ns = "balls";   //name of markers
	ball_list.action = visualization_msgs::Marker::ADD;
	ball_list.pose.position.x=0; //the transformation between the frame and camera data, just set it (0,0,0,0,0,0) for (x,y,z,roll,pitch,yaw)
	ball_list.pose.position.y=0;
	ball_list.pose.position.z=0;
	ball_list.pose.orientation.x=0;
	ball_list.pose.orientation.y=0;
	ball_list.pose.orientation.z=0;
	ball_list.pose.orientation.w=1.0;

  ball_list.id = 0; //set the marker id. if you use another markers, then make them use their own unique ids
  ball_list.type = visualization_msgs::Marker::SPHERE_LIST;  //set the type of marker

  ball_list.scale.x=0.10; //set the radius of marker   1.0 means 1.0m, 0.001 means 1mm
  ball_list.scale.y=0.10;
  ball_list.scale.z=0.10;


     // for( size_t i = 0; i < contours_r.size(); i++ )
     // {
     //     //fits polygon on contour image
     //     approxPolyDP( contours_r[i], contours_r_poly[i], 3, true );
     //     //uses smallest polygon
     //     minEnclosingCircle( contours_r_poly[i], center_r[i],radius_r[i] );
     // }

     //keep increasing radius size from 0 to find all the blue balls
     for( size_t i = 0; i < contours_b.size(); i++ )
     {
         //fits polygon on contour image
         approxPolyDP( contours_b[i], contours_b_poly[i], 3, true );
         //uses smallest polygon
         minEnclosingCircle( contours_b_poly[i], center_b[i],
                             radius_b[i] );
     }
     for( size_t i = 0; i < contours_g.size(); i++ )
        {
            //fits polygon on contour image
            approxPolyDP( contours_g[i], contours_g_poly[i], 3, true );
            //uses smallest polygon
            minEnclosingCircle( contours_g_poly[i], center_g[i],
                                radius_g[i] );
        }

     //keep increasing red ball size from 0
     // for( size_t i = 0; i< contours_r.size(); i++ )
     // {
     //     //only apply the following code to balls with radius larger than iMin_tracking_ball_size
     //     //to make sure the code doesn't track unecessary small red spots
     //     if (radius_r[i] > iMin_tracking_ball_size)
     //     {
     //         Scalar color = Scalar( 0, 0, 255);
     //         //make contours of red balls with following parameters)
     //         drawContours( hsv_frame_red_canny, contours_r_poly,(int)i, color, 1, 8, vector<Vec4i>(), 0, Point() );
     //         vector<float> ball_position_r;
     //
     //         //store ball position with center and radius position (this will be used to display on screen)
     //         ball_position_r = pixel2point(center_r[i], radius_r[i]);
     //         float isx = ball_position_r[0];
     //         float isy = ball_position_r[1];
     //         float isz = ball_position_r[2];
     //         string sx = floatToString(isx);
     //         string sy = floatToString(isy);
     //         string sz = floatToString(isz);
     //         text = "Red ball:" + sx + "," + sy + "," + sz;
     //
     //         //write description text
     //         putText(result, text, center_r[i],1,1,Scalar(0,255,0),2);
     //         //actually draws circle
     //         circle( result, center_r[i], (int)radius_r[i], color, 2,8, 0 );
     //     }
     // }

     //same thing as above with blue ball
     if (contours_b.size() ==0){
       msg.img_x.resize(1);
       msg.img_y.resize(1);
       msg.img_r.resize(1);
       msg.img_x[0] = 0;
       msg.img_y[0] = 0;
       msg.img_r[0] = 0;
     }
     for( size_t i = 0; i< contours_b.size(); i++ )
     {
         //track blue balls with radius larger than iMin_tracking_ball_size
         if(radius_b[i] > iMin_tracking_ball_size)
         {
             Scalar color = Scalar( 255, 0, 0);
             //draw contours of blue balls with following parameters)
             drawContours( hsv_frame_blue_canny, contours_b_poly,
                           (int)i, color, 1, 8, vector<Vec4i>(), 0, Point() );
             vector<float> ball_position_b;
             //store information of ball positions with center and radius
             ball_position_b = pixel2point(center_b[i], radius_b[i]);
             float isx = ball_position_b[0];
             float isy = ball_position_b[1];
             float isz = ball_position_b[2];
             string sx = floatToString(isx);
             string sy = floatToString(isy);
             string sz = floatToString(isz);
             text = "Blue ball:" + sx + "," + sy + "," + sz;

             msg.img_x[i] = center_b[i].x;
             msg.img_y[i] = center_b[i].y;

             msg.img_r[i] = radius_b[i];


            geometry_msgs::Point p;
            p.x = isx;
            p.y = isy;
            p.z = isz;
            ball_list.points.push_back(p);

            std_msgs::ColorRGBA c;
            c.r = 0.0;
            c.g = 1.0;
            c.b = 0.0;
            c.a = 1.0;
            ball_list.colors.push_back(c);


             //write description text
             putText(result, text, center_b[i],2,1,Scalar(0,255,0),2);
             //actually draws circle
             circle( result, center_b[i], (int)radius_b[i], color, 2, 8, 0 );
         }
     }
     for( size_t i = 0; i< contours_g.size(); i++ )
        {
            //track green balls with radius larger than iMin_tracking_ball_size
            if(radius_g[i] > iMin_tracking_ball_size)
            {
                Scalar color = Scalar( 0, 255, 0);
                //draw contours of green balls with following parameters)
                drawContours( hsv_frame_green_canny, contours_g_poly,
                              (int)i, color, 1, 8, vector<Vec4i>(), 0, Point() );
                vector<float> ball_position_g;
                //store information of ball positions with center and radius
                ball_position_g = pixel2point(center_g[i], radius_g[i]);
                float isx = ball_position_g[0];
                float isy = ball_position_g[1];
                float isz = ball_position_g[2];
                string sx = floatToString(isx);
                string sy = floatToString(isy);
                string sz = floatToString(isz);
                text = "Green ball:" + sx + "," + sy + "," + sz;
                //write description text
                putText(result, text, center_g[i],1,1,Scalar(0,255,0),2);
                //actually draws circle
                circle( result, center_g[i], (int)radius_g[i], color, 2, 8, 0 );
            }
        }

     //cv::imshow("view", frame);  //show the image with a window
     cv::waitKey(1); //main purpose of this command is to wait for a key command. However, many highgui related functions like imshow, redraw, and resize can not work without this command...just use it!

     imshow("Video Capture",calibrated_frame);
     imshow("Object Detection_HSV_Green",hsv_frame_green);

     imshow("Canny Edge for Green Ball", hsv_frame_green_canny);
     // imshow("Object Detection_HSV_Red",hsv_frame_red);
     imshow("Object Detection_HSV_Blue",hsv_frame_blue);
     // imshow("Canny Edge for Red Ball", hsv_frame_red_canny);
     imshow("Canny Edge for Blue Ball", hsv_frame_blue_canny);
     imshow("Result", result);

     pub.publish(msg);  //publish a message
     pub_markers.publish(ball_list);  //publish a marker message

}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{


   if(msg->height==480){  //check the size of the image received. if the image have 640x480, then change the buffer size to 640x480.
	cv::resize(buffer,buffer,cv::Size(640,480));
}
   else{
	//do nothing!
   }

   try
   {
     buffer = cv_bridge::toCvShare(msg, "bgr8")->image;  //transfer the image data into buffer
   }
   catch (cv_bridge::Exception& e)
   {
     ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
   }
   ball_detect(); //proceed ball detection
}


int main(int argc, char **argv)
{

      //Make windows for user interface to be shown on computer screen)
      namedWindow("Video Capture", WINDOW_NORMAL);
      namedWindow("Object Detection_HSV_Green", WINDOW_NORMAL);
      namedWindow("Canny Edge for Green Ball", WINDOW_NORMAL);
      namedWindow("Object Detection_HSV_Blue", WINDOW_NORMAL);
      namedWindow("Canny Edge for Blue Ball", WINDOW_NORMAL);

      // namedWindow("Canny Edge for Red Ball", WINDOW_NORMAL);
      // namedWindow("Object Detection_HSV_Red", WINDOW_NORMAL);

      namedWindow("Result", WINDOW_NORMAL);


      moveWindow("Video Capture",50, 0);

      moveWindow("Object Detection_HSV_Green",50,370);
    	moveWindow("Canny Edge for Green Ball",50,730);

      // moveWindow("Object Detection_HSV_Red", 50,370);
      // moveWindow("Canny Edge for Red Ball",50,730);

      // moveWindow("Object Detection_HSV_Blue",910,370);
      // moveWindow("Canny Edge for Red Ball",910,730);
      //Move the windows made above to specific locations on the screen so that they don't overlap
      moveWindow("Object Detection_HSV_Blue",470,370);
      moveWindow("Canny Edge for Blue Ball", 470,730);
      moveWindow("Result", 470, 0);

      // Trackbars to set thresholds for HSV values : Red ball
      // createTrackbar("Low H","Object Detection_HSV_Red", &low_h_r, 180, on_low_h_thresh_trackbar_red);
      // createTrackbar("High H","Object Detection_HSV_Red", &high_h_r, 180, on_high_h_thresh_trackbar_red);
      // createTrackbar("Low H2","Object Detection_HSV_Red", &low_h2_r, 180, on_low_h2_thresh_trackbar_red);
      // createTrackbar("High H2","Object Detection_HSV_Red", &high_h2_r, 180,on_high_h2_thresh_trackbar_red);
      // createTrackbar("Low S","Object Detection_HSV_Red", &low_s_r, 255,on_low_s_thresh_trackbar_red);
      // createTrackbar("High S","Object Detection_HSV_Red", &high_s_r, 255, on_high_s_thresh_trackbar_red);
      // createTrackbar("Low V","Object Detection_HSV_Red", &low_v_r, 255, on_low_v_thresh_trackbar_red);
      // createTrackbar("High V","Object Detection_HSV_Red", &high_v_r, 255, on_high_v_thresh_trackbar_red);

      // Trackbars to set thresholds for HSV values : Blue ball
      createTrackbar("Low H","Object Detection_HSV_Blue", &low_h_b, 180, on_low_h_thresh_trackbar_blue);
      createTrackbar("High H","Object Detection_HSV_Blue", &high_h_b, 180, on_high_h_thresh_trackbar_blue);
      createTrackbar("Low S","Object Detection_HSV_Blue", &low_s_b, 255, on_low_s_thresh_trackbar_blue);
      createTrackbar("High S","Object Detection_HSV_Blue", &high_s_b, 255, on_high_s_thresh_trackbar_blue);
      createTrackbar("Low V","Object Detection_HSV_Blue", &low_v_b, 255, on_low_v_thresh_trackbar_blue);
      createTrackbar("High V","Object Detection_HSV_Blue", &high_v_b, 255, on_high_v_thresh_trackbar_blue);


      createTrackbar("Low H","Object Detection_HSV_Green", &low_h_g, 180, on_low_h_thresh_trackbar_green);
    	createTrackbar("High H","Object Detection_HSV_Green", &high_h_g, 180, on_high_h_thresh_trackbar_green);
    	createTrackbar("Low S","Object Detection_HSV_Green", &low_s_g, 255,on_low_s_thresh_trackbar_green);
    	createTrackbar("High S","Object Detection_HSV_Green", &high_s_g, 255, on_high_s_thresh_trackbar_green);
    	createTrackbar("Low V","Object Detection_HSV_Green", &low_v_g, 255, on_low_v_thresh_trackbar_green);
    	createTrackbar("High V","Object Detection_HSV_Green", &high_v_g, 255, on_high_v_thresh_trackbar_green);
      // Trackbar to set parameter for Canny Edge
      // createTrackbar("Min Threshold:","Canny Edge for Red Ball", &lowThreshold_r, 100, on_canny_edge_trackbar_red);
      createTrackbar("Min Threshold:","Canny Edge for Blue Ball", &lowThreshold_b, 100, on_canny_edge_trackbar_blue);
      createTrackbar("Min Threshold:","Canny Edge for Green Ball", &lowThreshold_b, 100, on_canny_edge_trackbar_green);


   ros::init(argc, argv, "ball_detect_node2"); //init ros nodd
   ros::NodeHandle nh; //create node handler
   image_transport::ImageTransport it(nh); //create image transport and connect it to node hnalder
   image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback); //create subscriber

   pub = nh.advertise<core_msgs::ball_position>("/position", 100); //setting publisher
   pub_markers = nh.advertise<visualization_msgs::Marker>("/balls",1);

   ros::spin(); //spin.
   return 0;
}

//mainend

//convert integers to string
string intToString(int n)
{
    stringstream s;
    s << n;
    return s.str();
}

//convert float values to string
string floatToString(float f)
{
    ostringstream buffer;
    // buffer << std::setprecision(std::numeric_limits<float>::digits10+2);
    buffer << f;
    return buffer.str();
}

//morph0ps filter for image smoothing
//erode-dilate computation is usually used to combine multiple small images to one large image
void morphOps(Mat &thresh)
{
    //create structuring element that will be used to "dilate" and "erode" image.
    //the element chosen here is a 3px by 3px rectangle
    Mat erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));
    //dilate with larger element so make sure object is nicely visible
    Mat dilateElement = getStructuringElement( MORPH_RECT,Size(8,8));
    //erode changes colors to darkest (lowest) value within filter
    erode(thresh,thresh,erodeElement);
    erode(thresh,thresh,erodeElement);
    //dilate changes colors to brighest (highest) value within filter
    dilate(thresh,thresh,dilateElement);
    dilate(thresh,thresh,dilateElement);
}


//function used to store center and radius information for position of balls
vector<float> pixel2point(Point center, int radius)
{
    //parameters and informations that will be stored in this function (declaration)
    vector<float> position;
    float x, y, u, v, Xc, Yc, Zc;

    //there are x and y centers because the screen is 2D with pixel values
    x = center.x;//.x;// .at(0);

    y = center.y;//.y;//


    //values used to calculate 3D iformation with 2D information
    u = (x-intrinsic_data[2])/intrinsic_data[0];
    v = (y-intrinsic_data[5])/intrinsic_data[4];

    //convert x and y values to Xc and Yc values to calculate Zc, which is distance or depth of ball within screen

    Zc = (intrinsic_data[0]*fball_radius)/(2*(float)radius);
    Xc=u*Zc;
    Yc =v*Zc;
    //round foat values to third decimal place

    Xc =roundf(Xc * 1000) / 1000;
    Yc =roundf(Yc * 1000) / 1000;
    Zc= roundf(Zc * 1000) / 1000;


    // Xc = Xc + 0.332;
    // Yc = Yc + 0.2;

    //save data at position and return data
    position.push_back(Xc);
    position.push_back(Yc);
    position.push_back(Zc);

    return position;
}


// Trackbar for image threshodling in HSV colorspace : Red
// void on_low_h_thresh_trackbar_red(int, void *)
// {
//     low_h_r = min(high_h_r-1, low_h_r);
//     setTrackbarPos("Low H","Object Detection_HSV_Red", low_h_r);
// }
// void on_high_h_thresh_trackbar_red(int, void *)
// {
//     high_h_r = max(high_h_r, low_h_r+1);
//     setTrackbarPos("High H", "Object Detection_HSV_Red", high_h_r);
// }
// void on_low_h2_thresh_trackbar_red(int, void *)
// {
//     low_h2_r = min(high_h2_r-1, low_h2_r);
//     setTrackbarPos("Low H2","Object Detection_HSV_Red", low_h2_r);
// }
// void on_high_h2_thresh_trackbar_red(int, void *)
// {
//     high_h_r = max(high_h_r, low_h_r+1);
//     setTrackbarPos("High H", "Object Detection_HSV_Red", high_h_r);
// }
// void on_low_s_thresh_trackbar_red(int, void *)
// {
//     low_s_r = min(high_s_r-1, low_s_r);
//     setTrackbarPos("Low S","Object Detection_HSV_Red", low_s_r);
// }
// void on_high_s_thresh_trackbar_red(int, void *)
// {
//     high_s_r = max(high_s_r, low_s_r+1);
//     setTrackbarPos("High S", "Object Detection_HSV_Red", high_s_r);
// }
// void on_low_v_thresh_trackbar_red(int, void *)
// {
//     low_v_r= min(high_v_r-1, low_v_r);
//     setTrackbarPos("Low V","Object Detection_HSV_Red", low_v_r);
// }
// void on_high_v_thresh_trackbar_red(int, void *)
// {
//     high_v_r = max(high_v_r, low_v_r+1);
//     setTrackbarPos("High V", "Object Detection_HSV_Red", high_v_r);
// }


// Trackbar for image threshodling in HSV colorspace : Blue
void on_low_h_thresh_trackbar_blue(int, void *)
{
    low_h_b = min(high_h_b-1, low_h_b);
    setTrackbarPos("Low H","Object Detection_HSV_Blue", low_h_b);
}
void on_high_h_thresh_trackbar_blue(int, void *)
{
    high_h_b = max(high_h_b, low_h_b+1);
    setTrackbarPos("High H", "Object Detection_HSV_Blue", high_h_b);
}
void on_low_s_thresh_trackbar_blue(int, void *)
{
    low_s_b = min(high_s_b-1, low_s_b);
    setTrackbarPos("Low S","Object Detection_HSV_Blue", low_s_b);
}
void on_high_s_thresh_trackbar_blue(int, void *)
{
    high_s_b = max(high_s_b, low_s_b+1);
    setTrackbarPos("High S", "Object Detection_HSV_Blue", high_s_b);
}
void on_low_v_thresh_trackbar_blue(int, void *)
{
    low_v_b= min(high_v_b-1, low_v_b);
    setTrackbarPos("Low V","Object Detection_HSV_Blue", low_v_b);
}
void on_high_v_thresh_trackbar_blue(int, void *)
{
    high_v_b = max(high_v_b, low_v_b+1);
    setTrackbarPos("High V", "Object Detection_HSV_Blue", high_v_b);
}

void on_low_h_thresh_trackbar_green(int, void *)
{
    low_h_g = min(high_h_g-1, low_h_g);
    setTrackbarPos("Low H","Object Detection_HSV_Green", low_h_g);
}
void on_high_h_thresh_trackbar_green(int, void *)
{
    high_h_g = max(high_h_g, low_h_g+1);
    setTrackbarPos("High H", "Object Detection_HSV_Green", high_h_g);
}
void on_low_s_thresh_trackbar_green(int, void *)
{
    low_s_g = min(high_s_g-1, low_s_g);
    setTrackbarPos("Low S","Object Detection_HSV_Green", low_s_g);
}
void on_high_s_thresh_trackbar_green(int, void *)
{
    high_s_g = max(high_s_g, low_s_g+1);
    setTrackbarPos("High S", "Object Detection_HSV_Green", high_s_g);
}
void on_low_v_thresh_trackbar_green(int, void *)
{
    low_v_g= min(high_v_g-1, low_v_g);
    setTrackbarPos("Low V","Object Detection_HSV_Green", low_v_g);
}
void on_high_v_thresh_trackbar_green(int, void *)
{
    high_v_g = max(high_v_g, low_v_g+1);
    setTrackbarPos("High V", "Object Detection_HSV_Green", high_v_g);
}

void on_canny_edge_trackbar_green(int, void *)
{
    setTrackbarPos("Min Threshold", "Canny Edge for Green Ball",lowThreshold_g);
}






// Trackbar for Canny edge algorithm
// void on_canny_edge_trackbar_red(int, void *)
// {
//     setTrackbarPos("Min Threshold", "Canny Edge for Red Ball",lowThreshold_r);
// }
void on_canny_edge_trackbar_blue(int, void *)
{
    setTrackbarPos("Min Threshold", "Canny Edge for Blue Ball",lowThreshold_b);
}
