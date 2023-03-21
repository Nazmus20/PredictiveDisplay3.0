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


//Store the PTZ image
void imageCb_ptz(const sensor_msgs::ImageConstPtr& msg, Parameters* P)
{ 
    cv_bridge::CvImagePtr cv_ptr;

   //Get time header info
    std_msgs::Header h = msg->header;
    (*P).t0_PTZ = double(h.stamp.sec) + 1e-9*double(h.stamp.nsec);
    
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

	(*P).ptz_delay = cv_ptr->image;
}

//Store the Omni image
void imageCb_omni(const sensor_msgs::ImageConstPtr& msg, Parameters* P)
{
    cv_bridge::CvImagePtr cv_ptr;
    //Get time header info
    std_msgs::Header h = msg->header;
    (*P).t0_omni = double(h.stamp.sec) + 1e-9*double(h.stamp.nsec);
    
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    (*P).omni_delay = cv_ptr->image;
}

//Store the currently commanded gimble angle in the right parameter
void IMUAngleCb(const boost::shared_ptr<const geometry_msgs::TwistStamped>& msg, Parameters* P)
{
/*
        //Delayed gimbal IMU angles and their timestamps
        (*P).t_imu_cur = (*msg).header.stamp.sec + 1e-9 * (*msg).header.stamp.nsec;
        //std::cout << (time - (*P).t0_PTZ) * 1000 << std::endl<<std::endl;
        
        //Quaternion to be converted to angles
        double q_x = (*msg).pose.orientation.x;
        double q_y = (*msg).pose.orientation.y;
        double q_z = (*msg).pose.orientation.z;
        double q_w = (*msg).pose.orientation.w;    
        
        //Roll (z-axis rotation of the camera)
        double sinr_cosp = 2 * (q_w * q_x + q_y * q_z);
        double cosr_cosp = 1 - 2 * (q_x * q_x + q_y * q_y);
        (*P).roll_imu_cur = atan2(sinr_cosp, cosr_cosp); //rad, will remain constant
        
        //Yaw (y-axis rotation of the camera)
        double siny_cosp = +2.0 * (q_w * q_z + q_x * q_y);
        double cosy_cosp = +1.0 - 2.0 * (q_y * q_y + q_z * q_z);
        (*P).yaw_imu_cur = atan2(siny_cosp, cosy_cosp); //rad, need to zero it out
        //Pitch (x-axis rotation of the camera)
        double sinp = 2 * (q_w * q_y - q_z * q_x); //rad
        double pitch;
        if (abs(sinp) >= 1)
            (*P).pitch_imu_cur = copysign((*P).pi / 2, sinp); //use 90 deg if out of range, rad
        else
            (*P).pitch_imu_cur = asin(sinp); //rad
            */
                        
        (*P).t_new = (*msg).header.stamp.sec + 1e-9 * (*msg).header.stamp.nsec;
        (*P).pitch_new = (*msg).twist.angular.x; 
        (*P).yaw_new = (*msg).twist.angular.y;
        (*P).roll_new = (*msg).twist.angular.z;        
}

int main(int argc, char** argv){
    
    ros::init(argc,argv,"image_prediction");
    ros::NodeHandle node;

    Parameters P; //Creating an instance of class Param
    //P.begin_alg = ros::Time::now().toSec(); //For estimating the algorithm run-time
    int predictor_rate; //Run the predictor ar 30 Hz, independent of camera FPS
    node.getParam("/predictor_rate", predictor_rate);
    double predictor_duration = 1/predictor_rate;
    
    //Get the predictor and stitching boolean from global ROS parameter server
    node.getParam("/predictor", P.PD);
    node.getParam("/stitch", P.stitch);
    std::cout << "Predictor: " << P.PD << " Stitch: " << P.stitch << std::endl;    
    
    if(P.PD == true) //Whether you want the predictor or not; will require more information like commanded angles etc...
    {
        //New ROS subscriber
        ImageStitching IS; HomographyCalculation HC;
    
        //When replacing Pose Stamped with Twist Stamped
        ros::Subscriber sub_ang = node.subscribe<geometry_msgs::TwistStamped>("/commanded_angles", 1, boost::bind(&IMUAngleCb, _1, &P));
        ros::Subscriber sub_quat = node.subscribe<geometry_msgs::PoseStamped>("/delayed_imu_angles", 1, boost::bind(&HomographyCalculation::PoseCallback, &HC, _1, &P));
    
        //When replacing TwistStamped with PoseStamped
        //ros::Subscriber sub_ang = node.subscribe<geometry_msgs::PoseStamped>("/delayed_imu_angles", 1, boost::bind(&IMUAngleCb, _1, &P));
        //ros::Subscriber sub_quat = node.subscribe<geometry_msgs::TwistStamped>("/commanded_angles", 1, boost::bind(&HomographyCalculation::PoseCallback, &HC, _1, &P));
    
        // Image subscriber should use different node name and subscriber.
        image_transport::ImageTransport it_(node);
        //image_transport::TransportHints hints("compressed", ros::TransportHints()); // To interpret compressed video not required here as the videos are already uncompressed
        
        
        //To store PTZ and Omni images in the global "parameter" variables to be used by other functions
        image_transport::Subscriber image_sub_omni = it_.subscribe("/omni_delay", 1, boost::bind(&imageCb_omni, _1, &P));
        image_transport::Subscriber image_sub_ptz = it_.subscribe("/ptz_delay", 1, boost::bind(&imageCb_ptz, _1, &P));
        
        ros::Rate loop_rate(predictor_rate);
        while(ros::ok())
        {
            ros::spinOnce();
            loop_rate.sleep();
        }

        return 0;
    }
}
