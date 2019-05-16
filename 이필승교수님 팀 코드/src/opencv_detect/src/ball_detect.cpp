#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <cv_bridge/cv_bridge.h>
#include "core_msgs/ball_position.h"
#include "opencv2/opencv.hpp"
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <iostream>
#include <string>
#include <stdlib.h>
#include <math.h>

using namespace cv;
using namespace std;

//////////////////////////////////////////////////////// Blue

void on_low_h_thresh_trackbar_blue(int, void *);
void on_high_h_thresh_trackbar_blue(int, void *);
void on_low_s_thresh_trackbar_blue(int, void *);
void on_high_s_thresh_trackbar_blue(int, void *);
void on_low_v_thresh_trackbar_blue(int, void *);
void on_high_v_thresh_trackbar_blue(int, void *);

int low_h_b=100, low_s_b=90, low_v_b=60;
int high_h_b=120, high_s_b=255, high_v_b=255;

void on_canny_edge_trackbar_blue(int, void *);

int lowThreshold_b = 100;
int ratio_b = 3;
int kernel_size_b = 3;

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

void on_canny_edge_trackbar_blue(int, void *)
{
setTrackbarPos("Min Threshold", "Canny Edge for Blue Ball", lowThreshold_b);
}

//////////////////////////////////////////////////////////

//////////////////////////////////////////////////////// Green

void on_low_h_thresh_trackbar_green(int, void *);
void on_high_h_thresh_trackbar_green(int, void *);
void on_low_s_thresh_trackbar_green(int, void *);
void on_high_s_thresh_trackbar_green(int, void *);
void on_low_v_thresh_trackbar_green(int, void *);
void on_high_v_thresh_trackbar_green(int, void *);

int low_h_g=60, low_s_g=40, low_v_g=30;
int high_h_g=90, high_s_g=255, high_v_g=255;

void on_canny_edge_trackbar_green(int, void *);

int lowThreshold_g = 100;
int ratio_g = 3;
int kernel_size_g = 3;

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
setTrackbarPos("Min Threshold", "Canny Edge for Green Ball", lowThreshold_g);
}

//////////////////////////////////////////////////////////

//////////////////////////////////////////////////////// Yellow

void on_low_h_thresh_trackbar_yellow(int, void *);
void on_high_h_thresh_trackbar_yellow(int, void *);
void on_low_s_thresh_trackbar_yellow(int, void *);
void on_high_s_thresh_trackbar_yellow(int, void *);
void on_low_v_thresh_trackbar_yellow(int, void *);
void on_high_v_thresh_trackbar_yellow(int, void *);

int low_h_y=10, low_s_y=120, low_v_y=150;
int high_h_y=30, high_s_y=255, high_v_y=255;

void on_canny_edge_trackbar_yellow(int, void *);

int lowThreshold_y = 100;
int ratio_y = 3;
int kernel_size_y = 3;

void on_low_h_thresh_trackbar_yellow(int, void *)
{
low_h_y = min(high_h_y-1, low_h_y);
setTrackbarPos("Low H","Object Detection_HSV_Yellow", low_h_y);
}
void on_high_h_thresh_trackbar_yellow(int, void *)
{
high_h_y = max(high_h_y, low_h_y+1);
setTrackbarPos("High H", "Object Detection_HSV_Yellow", high_h_y);
}
void on_low_s_thresh_trackbar_yellow(int, void *)
{
low_s_y = min(high_s_y-1, low_s_y);
setTrackbarPos("Low S","Object Detection_HSV_Yellow", low_s_y);
}
void on_high_s_thresh_trackbar_yellow(int, void *)
{
high_s_y = max(high_s_y, low_s_y+1);
setTrackbarPos("High S", "Object Detection_HSV_Yellow", high_s_y);
}
void on_low_v_thresh_trackbar_yellow(int, void *)
{
low_v_y= min(high_v_y-1, low_v_y);
setTrackbarPos("Low V","Object Detection_HSV_Yellow", low_v_y);
}
void on_high_v_thresh_trackbar_yellow(int, void *)
{
high_v_y = max(high_v_y, low_v_y+1);
setTrackbarPos("High V", "Object Detection_HSV_Yellow", high_v_y);
}

void on_canny_edge_trackbar_yellow(int, void *)
{
setTrackbarPos("Min Threshold", "Canny Edge for Yellow Ball", lowThreshold_y);
}

//////////////////////////////////////////////////////////


string intToString(int n);
string floatToString(float f);

void morphOps(Mat &thresh);

vector<float> pixel2point(Point center, int radius);

float fball_radius = 0.062 ; // meter

Mat distCoeffs;
// float intrinsic_data[9] = {596.497705,  0, 318.023172, 0, 565.010517, 271.337191, 0, 0, 1};
// float distortion_data[5] = {0.073823, -0.079246, 0.005644, -0.005499, 0};
float intrinsic_data[9] = {643.515946,  0, 323.895411, 0, 644.631585, 241.115684, 0, 0, 1};
float distortion_data[5] = {0.050308, -0.166614, 0.001681, -0.008035, 0};


double fontScale = 2;
int thickness = 3;
String text ;

int iMin_tracking_ball_size = 10;

string intToString(int n)
{
    stringstream s;
    s << n;
    return s.str();
}

string floatToString(float f)
{
    ostringstream buffer;
    buffer << f;
    return buffer.str();
}

void morphOps(Mat &thresh)
{

    Mat erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));
    Mat dilateElement = getStructuringElement( MORPH_RECT,Size(8,8));

    erode(thresh,thresh,erodeElement);
    erode(thresh,thresh,erodeElement);

    dilate(thresh,thresh,dilateElement);
    dilate(thresh,thresh,dilateElement);
}

vector<float> pixel2point(Point center, int radius)
{

    vector<float> position;
    float x, y, u, v, Xc, Yc, Zc;

	  x = center.x;
    y = center.y;

	  u = (x-intrinsic_data[2])/intrinsic_data[0];
    v = (y-intrinsic_data[5])/intrinsic_data[4];

    Zc = (intrinsic_data[0]*fball_radius)/(2*(float)radius) ;
    Xc = u*Zc ;
    Yc = v*Zc ;

    Xc = roundf(Xc * 1000) / 1000;
    Yc = roundf(Yc * 1000) / 1000;
    Zc = roundf(Zc * 1000) / 1000;

    position.push_back(Xc);
    position.push_back(Yc);
    position.push_back(Zc);

    return position;
}


 Mat buffer(1280,960,CV_8UC1);
//Mat buffer(1080,1440,CV_8UC1);
ros::Publisher pub;
ros::Subscriber data_sub;



void ball_detect()
{

        Mat frame;  //assign a memory to save the images

        // if(buffer.size().width==320){ //if the size of the image is 320x240, then resized it to 640x480
        //     cv::resize(buffer, frame, cv::Size(480, 640));
        // }
        // else{
             frame = buffer;
        // }
    ///////////////////////////////////////////////////////////
        Mat bgr_frame, hsv_frame,
        hsv_frame_blue, hsv_frame_blue_blur, hsv_frame_blue_canny,
        hsv_frame_green, hsv_frame_green_blur, hsv_frame_green_canny,
        hsv_frame_yellow, hsv_frame_yellow_blur, hsv_frame_yellow_canny,
        result;


        Mat calibrated_frame;
        Mat intrinsic = Mat(3,3, CV_32FC1);
        Mat distCoeffs;

        intrinsic = Mat(3, 3, CV_32F, intrinsic_data);
        distCoeffs = Mat(1, 5, CV_32F, distortion_data);

        vector<Vec4i> hierarchy_b;
        vector<Vec4i> hierarchy_g;
        vector<Vec4i> hierarchy_y;

        vector<vector<Point> > contours_b;
        vector<vector<Point> > contours_g;
        vector<vector<Point> > contours_y;

        //VideoCapture cap(0);


    	  // namedWindow("Video Capture", WINDOW_NORMAL);
        // namedWindow("Object Detection_HSV_Blue", WINDOW_NORMAL);
        // namedWindow("Canny Edge for Blue Ball", WINDOW_NORMAL);
        // namedWindow("Object Detection_HSV_Green", WINDOW_NORMAL);
        // namedWindow("Canny Edge for Green Ball", WINDOW_NORMAL);
        // namedWindow("Object Detection_HSV_Yellow", WINDOW_NORMAL);
        // namedWindow("Canny Edge for Yellow Ball", WINDOW_NORMAL);
        // namedWindow("Result", WINDOW_NORMAL);

        namedWindow("Video Capture", WINDOW_NORMAL);
        namedWindow("Object Detection_HSV_Blue", WINDOW_NORMAL);
        namedWindow("Canny Edge for Blue Ball", WINDOW_NORMAL);
        namedWindow("Object Detection_HSV_Green", WINDOW_NORMAL);
        namedWindow("Canny Edge for Green Ball", WINDOW_NORMAL);
        namedWindow("Object Detection_HSV_Yellow", WINDOW_NORMAL);
        namedWindow("Canny Edge for Yellow Ball", WINDOW_NORMAL);
        namedWindow("Result", WINDOW_AUTOSIZE);


        moveWindow("Video Capture",             50,  0);
        moveWindow("Object Detection_HSV_Blue",50,370);
        moveWindow("Canny Edge for Blue Ball", 50,730);
        moveWindow("Object Detection_HSV_Green",470,370);
        moveWindow("Canny Edge for Green Ball", 470,730);
        moveWindow("Object Detection_HSV_Yellow",890,370);
        moveWindow("Canny Edge for Yellow Ball", 890,730);
        moveWindow("Result", 50, 0);

        createTrackbar("Low H","Object Detection_HSV_Blue", &low_h_b, 180,
        on_low_h_thresh_trackbar_blue);
        createTrackbar("High H","Object Detection_HSV_Blue", &high_h_b, 180,
        on_high_h_thresh_trackbar_blue);
        createTrackbar("Low S","Object Detection_HSV_Blue", &low_s_b, 255,
        on_low_s_thresh_trackbar_blue);
        createTrackbar("High S","Object Detection_HSV_Blue", &high_s_b, 255,
        on_high_s_thresh_trackbar_blue);
        createTrackbar("Low V","Object Detection_HSV_Blue", &low_v_b, 255,
        on_low_v_thresh_trackbar_blue);
        createTrackbar("High V","Object Detection_HSV_Blue", &high_v_b, 255,
        on_high_v_thresh_trackbar_blue);
        createTrackbar("Min Threshold:","Canny Edge for Blue Ball", &lowThreshold_b, 100,
        on_canny_edge_trackbar_blue);

        createTrackbar("Low H","Object Detection_HSV_Green", &low_h_g, 180,
        on_low_h_thresh_trackbar_green);
        createTrackbar("High H","Object Detection_HSV_Green", &high_h_g, 180,
        on_high_h_thresh_trackbar_green);
        createTrackbar("Low S","Object Detection_HSV_Green", &low_s_g, 255,
        on_low_s_thresh_trackbar_green);
        createTrackbar("High S","Object Detection_HSV_Green", &high_s_g, 255,
        on_high_s_thresh_trackbar_green);
        createTrackbar("Low V","Object Detection_HSV_Green", &low_v_g, 255,
        on_low_v_thresh_trackbar_green);
        createTrackbar("High V","Object Detection_HSV_Green", &high_v_g, 255,
        on_high_v_thresh_trackbar_green);
        createTrackbar("Min Threshold:","Canny Edge for Green Ball", &lowThreshold_g, 100,
        on_canny_edge_trackbar_green);

        createTrackbar("Low H","Object Detection_HSV_Yellow", &low_h_y, 180,
        on_low_h_thresh_trackbar_yellow);
        createTrackbar("High H","Object Detection_HSV_Yellow", &high_h_y, 180,
        on_high_h_thresh_trackbar_yellow);
        createTrackbar("Low S","Object Detection_HSV_Yellow", &low_s_y, 255,
        on_low_s_thresh_trackbar_yellow);
        createTrackbar("High S","Object Detection_HSV_Yellow", &high_s_y, 255,
        on_high_s_thresh_trackbar_yellow);
        createTrackbar("Low V","Object Detection_HSV_Yellow", &low_v_y, 255,
        on_low_v_thresh_trackbar_yellow);
        createTrackbar("High V","Object Detection_HSV_Yellow", &high_v_y, 255,
        on_high_v_thresh_trackbar_yellow);
        createTrackbar("Min Threshold:","Canny Edge for Yellow Ball", &lowThreshold_y, 100,
        on_canny_edge_trackbar_yellow);


        core_msgs::ball_position msg;  //create a message for ball positions

        //while((char)waitKey(1)!='q') //
        //{


            //if(frame.empty())
                //break;


            undistort(frame, calibrated_frame, intrinsic, distCoeffs);

            result = calibrated_frame.clone();

            medianBlur(calibrated_frame, calibrated_frame, 3);

            cvtColor(calibrated_frame, hsv_frame, cv::COLOR_BGR2HSV);

            inRange(hsv_frame,Scalar(low_h_b,low_s_b,low_v_b),Scalar(high_h_b,high_s_b,high_v_b),hsv_frame_blue);
            inRange(hsv_frame,Scalar(low_h_g,low_s_g,low_v_g),Scalar(high_h_g,high_s_g,high_v_g),hsv_frame_green);
            inRange(hsv_frame,Scalar(low_h_y,low_s_y,low_v_y),Scalar(high_h_y,high_s_y,high_v_y),hsv_frame_yellow);


            morphOps(hsv_frame_blue);
            morphOps(hsv_frame_green);
            morphOps(hsv_frame_yellow);

            GaussianBlur(hsv_frame_blue, hsv_frame_blue_blur, cv::Size(9, 9), 2, 2);
            GaussianBlur(hsv_frame_green, hsv_frame_green_blur, cv::Size(9, 9), 2, 2);
            GaussianBlur(hsv_frame_yellow, hsv_frame_yellow_blur, cv::Size(9, 9), 2, 2);

            Canny(hsv_frame_blue_blur, hsv_frame_blue_canny, lowThreshold_b, lowThreshold_b*ratio_b, kernel_size_b);
            Canny(hsv_frame_green_blur, hsv_frame_green_canny, lowThreshold_g, lowThreshold_g*ratio_g, kernel_size_g);
            Canny(hsv_frame_yellow_blur, hsv_frame_yellow_canny, lowThreshold_y, lowThreshold_y*ratio_y, kernel_size_y);

            findContours(hsv_frame_blue_canny, contours_b, hierarchy_b, RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0, 0));
            findContours(hsv_frame_green_canny, contours_g, hierarchy_g, RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0, 0));
            findContours(hsv_frame_yellow_canny, contours_y, hierarchy_y, RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0, 0));


            vector<Point2f> imgxy;
            vector<Point2f> carf_xy;
            vector<Point2f> carb_xy;


            vector<vector<Point> > contours_b_poly( contours_b.size() );
            vector<vector<Point> > contours_g_poly( contours_g.size() );
            vector<vector<Point> > contours_y_poly( contours_y.size() );

            vector<Point2f>center_b( contours_b.size() );
            vector<Point2f>center_g( contours_g.size() );
            vector<Point2f>center_y( contours_y.size() );

            vector<float>radius_b( contours_b.size() );
            vector<float>radius_g( contours_g.size() );
            vector<float>radius_y( contours_y.size() );


            for( size_t i = 0; i < contours_b.size(); i++ )
            {
    			         approxPolyDP( contours_b[i], contours_b_poly[i], 3, true );
    			         minEnclosingCircle( contours_b_poly[i], center_b[i], radius_b[i] );
            }

            for( size_t i = 0; i< contours_b.size(); i++ )
            {

    			         if(radius_b[i] > iMin_tracking_ball_size)
                   {


                        Scalar color = Scalar( 255, 0, 0);

    				            drawContours( hsv_frame_blue_canny, contours_b_poly, (int)i, color, 1, 8, vector<Vec4i>(), 0, Point() );


                        string sx = floatToString(center_b[i].x);
                        string sy = floatToString(center_b[i].y);

                        text = "Blue ball" + intToString(i) + " : " + sx + "," + sy;

    				            //putText(result, text, center_b[i],2,1,Scalar(255, 0, 0),2);

    				            circle( result, center_b[i], (int)radius_b[i], color, 2, 8, 0 );


                        //cv::waitKey(1); //main purpose of this command is to wait for a key command. However, many highgui related functions like imshow, redraw, and resize can not work without this command...just use it!

                        imgxy.push_back(center_b[i]);

                    }
            }

            /////////////////////////////////

            for( size_t i = 0; i < contours_g.size(); i++ )
            {
    			         approxPolyDP( contours_g[i], contours_g_poly[i], 3, true );
    			         minEnclosingCircle( contours_g_poly[i], center_g[i], radius_g[i] );
            }

            for( size_t i = 0; i< contours_g.size(); i++ )
            {

    			         if(radius_g[i] > iMin_tracking_ball_size)
                   {
                        Scalar color = Scalar( 0, 255, 0);

    				            drawContours( hsv_frame_green_canny, contours_g_poly, (int)i, color, 1, 8, vector<Vec4i>(), 0, Point() );


                        string sx = floatToString(center_g[i].x);
                        string sy = floatToString(center_g[i].y);

                        text = "Green ball" + intToString(i) + " : " + sx + "," + sy;

    				            //putText(result, text, center_g[i],2,1,Scalar(0,255,0),2);

    				            circle( result, center_g[i], (int)radius_g[i], color, 2, 8, 0 );


                        //cv::waitKey(1); //main purpose of this command is to wait for a key command. However, many highgui related functions like imshow, redraw, and resize can not work without this command...just use it!

                        carf_xy.push_back(center_g[i]);

                    }
            }

            ////////////////////////////

            for( size_t i = 0; i < contours_y.size(); i++ )
            {
    			         approxPolyDP( contours_y[i], contours_y_poly[i], 3, true );
    			         minEnclosingCircle( contours_y_poly[i], center_y[i], radius_y[i] );
            }

            for( size_t i = 0; i< contours_y.size(); i++ )
            {

    			         if(radius_y[i] > iMin_tracking_ball_size)
                   {
                        Scalar color = Scalar( 0, 255, 255);

    				            drawContours( hsv_frame_yellow_canny, contours_y_poly, (int)i, color, 1, 8, vector<Vec4i>(), 0, Point() );


                        string sx = floatToString(center_y[i].x);
                        string sy = floatToString(center_y[i].y);

                        text = "Yellow ball" + intToString(i) + " : " + sx + "," + sy;

    				            //putText(result, text, center_y[i],2,1,Scalar(0, 255, 255),2);

    				            circle( result, center_y[i], (int)radius_y[i], color, 2, 8, 0 );


                        //cv::waitKey(1); //main purpose of this command is to wait for a key command. However, many highgui related functions like imshow, redraw, and resize can not work without this command...just use it!

                        carb_xy.push_back(center_y[i]);

                    }
            }




            msg.size =imgxy.size(); //adjust the size of message. (*the size of message is varying depending on how many circles are detected)
            msg.img_x.resize(imgxy.size());  //adjust the size of array
            msg.img_y.resize(imgxy.size());  //adjust the size of array
            msg.car_front_x.resize(carf_xy.size());
            msg.car_front_y.resize(carf_xy.size());
            msg.car_back_x.resize(carb_xy.size());
            msg.car_back_y.resize(carb_xy.size());

            for( size_t i = 0; i < imgxy.size(); i++ )
            {

    			         msg.img_x[i]=imgxy[i].x;
                   msg.img_y[i]=imgxy[i].y;

            }

            for( size_t i = 0; i < carf_xy.size(); i++ )
            {

    			         msg.car_front_x[i]=carf_xy[i].x;
                   msg.car_front_y[i]=carf_xy[i].y;

            }

            for( size_t i = 0; i < carb_xy.size(); i++ )
            {

    			         msg.car_back_x[i]=carb_xy[i].x;
                   msg.car_back_y[i]=carb_xy[i].y;

            }




            //imshow("Video Capture",calibrated_frame);
            imshow("Object Detection_HSV_Blue",hsv_frame_blue);
            imshow("Canny Edge for Blue Ball", hsv_frame_blue_canny);
            imshow("Object Detection_HSV_Green",hsv_frame_green);
            imshow("Canny Edge for Green Ball", hsv_frame_green_canny);
            imshow("Object Detection_HSV_Yellow",hsv_frame_yellow);
            imshow("Canny Edge for Yellow Ball", hsv_frame_yellow_canny);
            imshow("Result", result);

            cv::waitKey(1);

            pub.publish(msg);  //publish a message

        //}
    ///////////////////////////////////////////////////////////

}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{


  //std::cout<<msg->height<<std::endl;

  // if(msg->height==480)
  // {  //check the size of the image received. if the image have 640x480, then change the buffer size to 640x480.
  // 	cv::resize(buffer,buffer,cv::Size(480,640));
  // }
    //  else
    //  {
  	// //do nothing!
    //  }

    //cv::resize(buffer,buffer,cv::Size(1440,1080));

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

void data_info(const core_msgs::ball_position msg)
{
  if(msg.size==0)
  {
    cout<<"No ball detected"<<endl;
  }
  else
  {
    std::cout<<"ball number="<<msg.size<<std::endl;

    for( size_t i = 0; i < msg.size; i++ )
    {
      std::cout<<"ball x"<<i<<" position ="<<msg.img_x[i]<< ","<<msg.img_y[i]<<std::endl;
    }
}
}


int main(int argc, char **argv)
{
   ros::init(argc, argv, "ball_detect_node_cv"); //init ros nodd
   ros::NodeHandle nh; //create node handler
   image_transport::ImageTransport it(nh); //create image transport and connect it to node hnalder
   image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback); //create subscriber

   pub = nh.advertise<core_msgs::ball_position>("/position", 100); //setting publisher

   data_sub = nh.subscribe("/position", 1, data_info);
   //pub_markers = nh.advertise<visualization_msgs::Marker>("/balls",1);

   ros::spin(); //spin.
   return 0;
}
