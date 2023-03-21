//Standard Libraries
#include <iostream>
#include <fstream>
#include <ctime>
#include <cmath>
#include "bits/time.h"
#include <stdio.h>
#include <chrono>

//OpenCV Libraries
#include <opencv2/opencv.hpp>
#include "opencv2/features2d.hpp"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/ocl.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/calib3d/calib3d.hpp>

//////////////////////////
//Changed from old version
//#include <opencv2/opencv.hpp>
//#include <opencv2/highgui.hpp>
//////////////////////////

#include <sensor_msgs/image_encodings.h>

//OpenCV CUDA Libraries
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>

//ROS Libraries
#include <geometry_msgs/Twist.h>
#include "std_msgs/Float64.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

//To publish ROS clock topic
#include "rosgraph_msgs/Clock.h"

//To introduce delay using ROS
#include "message_filters/time_sequencer.h"
#include "message_filters/subscriber.h"
#include <std_msgs/String.h>

//Add the custom headers
#include "ImageStitching.h"
#include "HomographyCalculation.h"
#include "parameters.h"

//cv::Mat ptz_delay, omni_delay; //Delayed images
sensor_msgs::ImagePtr ptz_msg, omni_msg; //Store the image data for publishing

//Delays the PTZ image
void delayCbPTZ(const sensor_msgs::ImageConstPtr& msg, Parameters* P)
{ 
    cv_bridge::CvImagePtr cv_ptr;

   //Get time header info
    std_msgs::Header h = msg->header;
    //(*P).t0_PTZ = double(h.stamp.sec) + 1e-9*double(h.stamp.nsec);
    
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

	cv::Mat src_ptz; //Need to move the map out into parameters
	src_ptz = cv_ptr->image;
	
	//Undistort PTZ image
        cv::remap(src_ptz, (*P).ptz_delay, (*P).ptz_map1, (*P).ptz_map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0); 
	ptz_msg = cv_bridge::CvImage(h, "bgr8", (*P).ptz_delay).toImageMsg();
}

//Delays the Omni image
void delayCbOmni(const sensor_msgs::ImageConstPtr& msg, Parameters* P)
{
    cv_bridge::CvImagePtr cv_ptr;

    //Get time header info
    std_msgs::Header h = msg->header;
    //(*P).t0_omni = double(h.stamp.sec) + 1e-9*double(h.stamp.nsec);
    
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    cv::Mat src_omni = cv_ptr->image;
    
    // Crop image
    cv::Rect myROI = cv::Rect(0, 0, 1504, 1504);
    cv::Mat OmniImgDist(src_omni, myROI);
    
    // Undistort image
    cv::Mat omni_img_und;
    
    cv::remap(OmniImgDist, omni_img_und, (*P).omni_map1, (*P).omni_map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);
        
    // Flip image
    cv::flip(omni_img_und, (*P).omni_delay, -1);
    omni_msg = cv_bridge::CvImage(h, "bgr8", (*P).omni_delay).toImageMsg();
}

int main(int argc, char** argv){
    
    ros::init(argc,argv,"image_delay");
    ros::NodeHandle node;
    double delay3; //double delay4;
    bool predictor, stitch;
    int FPS;
/*    
    double delay3 = 0.0; //Delay in the omnicam image
    double delay4 = 0.0; //Delay in the PTZ image
    int FPS = 30; //Default frame rate of the cameras to be published
    bool predictor = false; //Boolean to turn the predictor on/off
    bool stitch = false; //Boolean to turn the stitcher on/off
    double duration = 1/FPS;
   
    node.setParam<int>("camera_fps", FPS);
    node.setParam<double>("ptz_delay", delay4);
    node.setParam<double>("omni_delay", delay3);
    node.setParam<bool>("predictor", predictor);
    node.setParam<bool>("stitch", stitch);
*/  
    node.getParam("camera_fps", FPS);
    node.getParam("ptz_delay", delay3);
    //node.getParam("omni_delay", delay4);
    node.getParam("predictor", predictor);
    node.getParam("stitch", stitch);    
    double duration = 1/FPS;
    if( predictor == false && stitch != false)
    {
        node.setParam("stitch", stitch);
    }
    
    
    //For debugging
    //std::cout << "Delay Params:\n" << std::endl;
    //std::cout << "Rate/FPS:\t" << FPS << std::endl;
    //std::cout << "PTZ delay:\t" << delay3 << std::endl;
    //std::cout << "Omni delay:\t" << delay4 << std::endl;
    //std::cout << "Prediction algorithm:\t" << predictor << std::endl;
    //std::cout << "Image stitching:\t" << stitch << std::endl;
    
    
    Parameters P; //Creating an instance of class Param
    //*********************************************************************************************************************************************************
    //Need to compute these only ONCE!!!!!!! The MOST time consuming step are these two lines below
    cv::initUndistortRectifyMap(P.Kp, P.Dp, cv::noArray(), P.Kp_new, cv::Size(1920, 1080), 0, P.ptz_map1, P.ptz_map2); //Assuming 1920X1080 image 
    cv::fisheye::initUndistortRectifyMap(P.Ko, P.Do, cv::noArray(), P.Ko, cv::Size(1504, 1504), CV_16SC2, P.omni_map1, P.omni_map2); //Assuming 1504X1504 image
    //*********************************************************************************************************************************************************
    //Change
    if(predictor == true) //Whether you want the predictor or not; will require more information like omnicam image
    {
        //To add delays to Omni
        message_filters::Subscriber<sensor_msgs::Image> sub_delay_omni(node, "/omnicam/omnicam/image_raw", 1);
        message_filters::TimeSequencer<sensor_msgs::Image> seq_delay_omni(sub_delay_omni, ros::Duration(delay3), ros::Duration(duration), 1 + int(FPS * delay3));
        seq_delay_omni.registerCallback(boost::bind(&delayCbOmni, _1, &P));
        
        //To add delays to PTZ, PTZ cam will be delayed with or without the Predictor active
        message_filters::Subscriber<sensor_msgs::Image> sub_delay_ptz(node, "/PTZcam/PTZcam/image_raw", 1);
        message_filters::TimeSequencer<sensor_msgs::Image> seq_delay_ptz(sub_delay_ptz, ros::Duration(delay3), ros::Duration(duration), 1 + int(FPS * delay3));
        seq_delay_ptz.registerCallback(boost::bind(&delayCbPTZ, _1, &P));
        
        // Image publisher
        image_transport::ImageTransport it(node);
        
        image_transport::Publisher pub_ptz_delay = it.advertise("/ptz_delay", 1);
        image_transport::Publisher pub_omni_delay = it.advertise("/omni_delay", 1);
        
        ros::Rate loop_rate(FPS);
        while(ros::ok())
        {
            ros::spinOnce();
            //Publish the delayed images to be used by predictor
            pub_ptz_delay.publish(ptz_msg);
            pub_omni_delay.publish(omni_msg);
            loop_rate.sleep();
        }
        return 0;
    }
    else
    {
        //To add delays to PTZ, PTZ cam will be delayed with or without the Predictor active
        message_filters::Subscriber<sensor_msgs::Image> sub_delay_ptz(node, "/PTZcam/PTZcam/image_raw", 1);
        message_filters::TimeSequencer<sensor_msgs::Image> seq_delay_ptz(sub_delay_ptz, ros::Duration(delay3), ros::Duration(.3333), 1 + int(FPS * delay3));
        seq_delay_ptz.registerCallback(boost::bind(&delayCbPTZ, _1, &P));
    
        ros::Rate loop_rate(FPS);
        while(ros::ok())
        {
            ros::spinOnce();
            //Show the PTZ image, do not publish anything
            if(!P.ptz_delay.empty())
            {
                cv::namedWindow("Final", cv::WND_PROP_FULLSCREEN);
                cv::setWindowProperty("Final", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN); 
                cv::circle(P.ptz_delay, cv::Point(960, 540), 5, cv::Scalar(0, 0, 255), cv::FILLED);
    	    
                cv::imshow("Final", P.ptz_delay);
                cv::waitKey(1); 
            }
            loop_rate.sleep();
        }

        return 0;
    }
}


