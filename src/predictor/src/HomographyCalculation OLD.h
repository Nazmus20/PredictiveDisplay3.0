// HomographyCalculation.h
#pragma once
#ifndef HomographyCalculation_H
#define HomographyCalculation_H

//Standard Libraries
//#include <vector>

//OpenCV Libraries
//#include <opencv2/opencv.hpp>

//ROS Libraries
#include <geometry_msgs/QuaternionStamped.h>

/*
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
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
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
*/

//Custom headers
#include "parameters.h"

class HomographyCalculation
{
public:
    HomographyCalculation(); //Default constructor

    ~HomographyCalculation(); //Default destructor

    //Calculate homography based on gimbal IMU angles
    void homCalc(double pitch0, double pitch1, double yaw0, double yaw1, Parameters* P);

    void QuatCallback(const boost::shared_ptr<const geometry_msgs::QuaternionStamped>& msg, Parameters* P);  
};

#endif
