
//Standard Libraries
#include <iostream>
#include <fstream>
//#include <ctime>
//#include <cmath>
//#include "bits/time.h"
//#include <stdio.h>
//#include <chrono>
//#include <numeric>
//#include <algorithm>
#include <vector>
#include<typeinfo>

//For iterating through filesystem
#include <boost/filesystem.hpp>

//OpenCV Libraries
#include <opencv2/opencv.hpp>
//#include "opencv2/features2d.hpp"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
//#include <opencv2/imgproc.hpp>
//#include <opencv2/imgcodecs.hpp>
//#include <image_transport/image_transport.h>
//#include <cv_bridge/cv_bridge.h>
//#include <opencv2/core/ocl.hpp> 
//#include <opencv2/video/video.hpp>
//#include <opencv2/calib3d/calib3d.hpp>
//#include <sensor_msgs/image_encodings.h>

//OpenCV CUDA Libraries
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>

//ROS Libraries
#include <ros/ros.h>
//#include <sensor_msgs/Joy.h>
//#include <geometry_msgs/TwistStamped.h>
//#include "std_msgs/Float64.h"
//#include <nav_msgs/Odometry.h>
//#include <geometry_msgs/Point.h>
//#include <geometry_msgs/QuaternionStamped.h>
//#include <tf/transform_listener.h>
//#include <tf2/linearmath/Quaternion.h>
//#include <tf2_ros/transform_broadcaster.h>
//#include <visualization_msgs/marker.h>

//To publish ROS clock topic
//#include "rosgraph_msgs/Clock.h"

//To introduce delay using ROS
//#include "message_filters/time_sequencer.h"
//#include "message_filters/subscriber.h"

//Camera intrinsic matrix for PTZcam (Pre determined)
cv::Mat Kp = (cv::Mat_<double>(3, 3) << 2120.49,	0,	981.1,
    0,	2119.06,	535.16,
    0,	0,	1);
//Distortion vector for the PTZcam (Used for undistorting a PTZcam image)
cv::Mat Dp = (cv::Mat_<double>(1, 4) << -3.94952719e-01, -6.68450461e-01, 5.26731133e-04, 1.10059397e-03, 7.20450676);
    
//Camera intrinsic matrix for Omnicam (Pre determined)
cv::Mat Ko = (cv::Mat_<double>(3, 3) << 445.17984003885283, 0.0, 721.2172155119429, 0.0, 445.8816513187479, 721.9473150432297, 0.0, 0.0, 1.0);
//Distortion vector for the omnicam (Used for undistorting an Omnicam image)
cv::Mat Do = (cv::Mat_<double>(1, 4) << -0.017496068223756347, -0.003243014339251435, 0.000839223410670477, -0.0004219234265229322);

//Store the mouse clicks in two global variables
std::vector<cv::Point2f> Omni_click, PTZ_click;
std::vector<cv::Point2f> PTZ_click_act; //Storing the PTZ pixels after shifting them
//Rescale the pixel points
std::vector<cv::Point2f> PTZ_click_rescale, Omni_click_rescale;

int shifting_pixel_value; //Value to shift the PTZ images

//Image and image name to store both the PTZ and the Omni images
cv::Mat dst, cacheOmni, cachePTZ, cacheLine; std::string Img_name;

//Global mouse click counter to dral line
int click_counter = 0;

double scale = 1; //rescale image

void mouseCB(int event, int x, int y, int flags, void* userdata)
{    

     int* val = static_cast<int*> (userdata);
     if  ( event == cv::EVENT_LBUTTONDOWN )
     {
         //Increase the counter of mouse clicks. Make sure to click on separate images to get the line
         click_counter = click_counter +1; 
         if(*val == 0)
         {//std::cout << "Here: " << *val << std::endl;
             dst.copyTo(cacheOmni); //Save the previous image state to cache
             //click_cnt is used to alternate between PTZ and Omni images
             *val = 1;
             //Store the left click pixels
             Omni_click.push_back(cv::Point(x, y));
             //Create a circle at the pixel position
             cv::circle(dst, cv::Point(x,y), 2, cv::Scalar(0, 0, 255), 1, cv::LINE_8, 0);
             cv::imshow(Img_name, dst);
             std::cout << "Omni px: " << Omni_click.back() << std::endl;  
         }
         else
         {
             dst.copyTo(cachePTZ); //Save the previous image state to cache
             *val = 0;
             //Store the left click pixels
             PTZ_click.push_back(cv::Point(x, y));
             //Create a circle at the pixel position
             cv::circle(dst, cv::Point(x,y), 2, cv::Scalar(0, 0, 255), 1, cv::LINE_8, 0);
             cv::imshow(Img_name, dst);
             std::cout << "PTZ px: " << PTZ_click.back() << std::endl;  
         }
         if(click_counter%2 == 0)
         {
             cv::line(dst, Omni_click.back(), PTZ_click.back(), cv::Scalar(0, 0, 255), 1);
             cv::imshow(Img_name, dst);
         }
     }
     
     //Remove points and line using right-click
     if(event == cv::EVENT_RBUTTONDOWN)
     {
         //Decrease the counter     
         click_counter = click_counter - 1;
         if(*val == 1) // Remove point from Omnicam
         {
             std::cout << "Removed Omni Px: " << Omni_click.back() << std::endl; 
             Omni_click.erase(Omni_click.end()); 
             cacheOmni.copyTo(dst); //copy previously saved image to dst
             cv::imshow(Img_name, dst);
             *val = 0;
         }
         else //Remove point from PTZ cam
         {
             std::cout << "Removed PTZ Px: " << PTZ_click.back() << std::endl;
             PTZ_click.erase(PTZ_click.end());
             cachePTZ.copyTo(dst); //copy previously saved image to dst
             cv::imshow(Img_name, dst);             
             *val = 1;
         }
     }
     

     
}

void hom_calc(cv::Mat &Omni_img, cv::Mat &PTZ_img, std::string name)
{
    std::string curr_img_name(1, name[42]);
    curr_img_name = "Image" + curr_img_name;
    Img_name = curr_img_name;
    cv::Mat Omni_img_re, PTZ_img_re;
    
    cv::resize(Omni_img, Omni_img_re, cv::Size(Omni_img.cols/scale, Omni_img.rows/scale), 0);
    cv::resize(PTZ_img, PTZ_img_re, cv::Size(PTZ_img.cols/scale, PTZ_img.rows/scale), 0);
    
    int row_PTZ = PTZ_img_re.rows; int col_PTZ = PTZ_img_re.cols; //Height and width of PTZ
    int row_Omni = Omni_img_re.rows; int col_Omni = Omni_img_re.cols; //Height and width of Omni 
    
    //Copy both images onto a single image
    cv::Mat dst1(row_Omni, col_Omni+col_PTZ, CV_8UC3); //Omni is larger so its height is used
    dst1.copyTo(dst);
    //std::cout << dst.rows << "X" << dst.cols << std::endl;
    
    cv::Mat Omni_left_roi(dst, cv::Rect(0, 0, Omni_img_re.size().width, Omni_img_re.size().height));
    Omni_img_re.copyTo(Omni_left_roi);
    cv::Mat PTZ_right_roi(dst, cv::Rect(Omni_img_re.size().width, 0, PTZ_img_re.size().width,  
    PTZ_img_re.size().height));
    PTZ_img_re.copyTo(PTZ_right_roi);
    
    //Since Omni and PTZ images have been placed side-by-side, the PTZ image needs to be shifted by the Omni image width for homography calculation
    shifting_pixel_value = Omni_img_re.size().width;
    
    //Set up the counters inside the callback
    int click_cnt = 0;
    //Obtain the mouse clicks
    cv::imshow(Img_name, dst);
    cv::setMouseCallback(Img_name, mouseCB, static_cast<void *> (&click_cnt)); 
    cv::waitKey(0);
}

int main(int argc, char** argv)
{
    //Specify the directories of the calibration images (must contain only .png images)
    std::string directoryOmni = "/home/qb/predictive_display_2.0/omni/";
    std::string directoryPTZ = "/home/qb/predictive_display_2.0/PTZ/";
    std::vector<std::string> OmniImgName, PTZImgName;
    //For loop to iterate through the image names
    boost::filesystem::path p(directoryOmni);
    for (auto i = boost::filesystem::directory_iterator(p); i != boost::filesystem::directory_iterator(); i++)
    {
        if (!is_directory(i -> path())) //we eliminate directories
        {
            OmniImgName.push_back(directoryOmni+i->path().filename().string());
            PTZImgName.push_back(directoryPTZ+i->path().filename().string());
        }
        else
            continue;
    }
    
    cv::Mat OmniImg, PTZImg, map1, map2;
    //For loop to perform stereo calibration
    for(int i = 0; i < OmniImgName.size(); i++)
    {
        cv::Mat OmniImgUncropped = cv::imread(OmniImgName[i]);
    	// Crop Omnicam image
	cv::Rect myROI = cv::Rect(0, 0, 1504, 1504);
	cv::Mat OmniImgDist(OmniImgUncropped,myROI);
        //std::cout << OmniImgName[i] << std::endl;
        // Undistort Omnicam image
        
        cv::fisheye::undistortImage(OmniImgDist, OmniImg, Ko, Do, Ko, cv::Size(OmniImgDist.cols, OmniImgDist.rows));
	//Flip the omnicam Image
	cv::flip(OmniImg, OmniImg, -1);
		
    	cv::Mat PTZImgDist = cv::imread(PTZImgName[i]);
    	
    	//Undistort PTZ image
    	cv::undistort(PTZImgDist, PTZImg, Kp, Dp, cv::noArray()); 
    	//Get new camera Matrix without any distortion
    	cv::Mat Kp_new = cv::getOptimalNewCameraMatrix(Kp, Dp, cv::Size(PTZImgDist.cols, PTZImgDist.rows), 0.0, cv::Size(PTZImgDist.cols, PTZImgDist.rows), 0, true);
    	std::cout << Kp_new << std::endl;
    	cv::initUndistortRectifyMap(Kp, Dp, cv::noArray(), Kp_new, cv::Size(PTZImgDist.cols, PTZImgDist.rows), 0, map1, map2);
    	cv::remap(PTZImgDist, PTZImg, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);  
    
	hom_calc(OmniImg, PTZImg, OmniImgName[i]);
    	//cv::imshow(PTZImgName[i], PTZImg);

    	//cv::imshow(PTZImgName[i]+"Dist", PTZImgDist);
    	//cv::imshow(OmniImgName[i], OmniImg);
    	//cv::waitKey(0);
    	//cv::destroyAllWindows();
    }
    //Actual PTZ pixel values, after shifting	
    for(int j = 0; j<PTZ_click.size(); j++)
    {
	PTZ_click_act.push_back(cv::Point(PTZ_click[j].x-shifting_pixel_value, PTZ_click[j].y));
	//std::cout << PTZ_click[j] << PTZ_click_act[j] << std::endl;
	PTZ_click_rescale.push_back(cv::Point(PTZ_click_act[j].x * scale, PTZ_click[j].y * scale));
	Omni_click_rescale.push_back(cv::Point(Omni_click[j].x * scale, Omni_click[j].y * scale));
    }
    cv::Mat H2 = (cv::Mat_<double>(3, 3) << 4.673773247454604, -0.001399667839128126, -2575.037754563543, -0.2305844524419365, 4.480166076504719, -2478.361189980256,
    4.094195555141877e-05, -4.056408665521137e-05, 1);
 
    cv::Mat H = (cv::Mat_<double>(3, 3) << 3.662595664683535, 0.05440616409171908, -1999.161621659138, -0.2770712432149974, 3.76913088044113, -2018.638427077817,
   -0.0001093606810511352, -8.214723739695882e-05, 0.9999999999999999);

    //cv::Mat H = cv::findHomography(Omni_click_rescale, PTZ_click_rescale, cv::RANSAC, 5.0, cv::noArray(), 10000, .995);
    //std::cout<< H << std::endl; 
    cv::Mat DstImg, DSTIMG;
    cv::warpPerspective(OmniImg, DstImg, H2, PTZImg.size());
    cv::warpPerspective(OmniImg, DSTIMG, H, PTZImg.size());
    cv::imshow("Final", DstImg);
    cv::imwrite("FinalOld.png", DSTIMG);
    cv::imwrite("Final.png", DstImg);
    cv::imwrite("PTZ.png", PTZImg);
    cv::waitKey(0);
    return 0;
}
