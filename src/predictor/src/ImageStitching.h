// ImageStitching.h
#pragma once
#ifndef ImageStitching_H
#define ImageStitching_H

//OpenCV Libraries
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

//ROS Libraries
#include <sensor_msgs/image_encodings.h>
#include "std_msgs/Float64.h"

//Custom headers
#include "parameters.h"

class ImageStitching
{
public:
	ImageStitching(); //Default constructor

	~ImageStitching(); //Default destructor

	void alignImages(cv::Mat& im1, cv::Mat& im2, cv::Mat& h2, cv::Mat& h1, Parameters* P);

	void imageCb_omni(const sensor_msgs::ImageConstPtr& msg, Parameters* P);

	void imageCb_ptz(const sensor_msgs::ImageConstPtr& msg, Parameters& P);
};

#endif 
