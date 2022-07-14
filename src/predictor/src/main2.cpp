
//Standard Libraries
//#include <iostream>
//#include <fstream>
//#include <ctime>
//#include <cmath>
//#include "bits/time.h"
//#include <stdio.h>
//#include <chrono>
#include <numeric>
//#include <algorithm>
//#include <vector>

//OpenCV Libraries
#include <opencv2/opencv.hpp>
//#include "opencv2/features2d.hpp"
//#include <opencv2/core.hpp>
//#include <opencv2/highgui.hpp>
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
//#include <opencv2/cudaarithm.hpp>
//#include <opencv2/cudaimgproc.hpp>

//ROS Libraries
//#include <geometry_msgs/Twist.h>
//#include "std_msgs/Float64.h"
//#include <nav_msgs/Odometry.h>
//#include <geometry_msgs/point.h>
//#include <geometry_msgs/quaternionstamped.h>
//#include <tf/transform_listener.h>
//#include <tf2/linearmath/quaternion.h>
//#include <tf2_ros/transform_broadcaster.h>
//#include <visualization_msgs/marker.h>

//To publish ROS clock topic
//#include "rosgraph_msgs/Clock.h"

//To introduce delay using ROS
//#include "message_filters/time_sequencer.h"
//#include "message_filters/subscriber.h"
//#include <std_msgs/String.h>

//Include created class
#include "HistogramMatching.h"

int main(int argc, char** argv)
{
	cv::Mat img2 = cv::imread("/home/qb/predictive_display_2.0/src/predictor/src/image5.png");
	cv::Mat img1 = cv::imread("/home/qb/predictive_display_2.0/src/predictor/src/image.png");

	/*
	cv::Mat img2 = cv::imread("/home/qb/predictive_display_2.0/src/stitcher/src/image5.png"); 
	cv::Mat img1 = cv::imread("/home/qb/predictive_display_2.0/src/stitcher/src/image.png");
	
	HistogramMatching HM; //Creating an object of the 'HistogramMatching' class
	HM.histMatch(img1, img2);
	
	cv::imshow("Img1", img1);
	cv::waitKey(0);
	*/
	//This part of the code is for debugging and calculating runtime speed
	
	cv::imshow("Img1", img1);
	cv::waitKey(0);
	clock_t t;
	std::vector<float> t_vec;
	for (int iter = 0; iter < 1000; iter++)
	{
		t = clock();
		HistogramMatching HM; //Creating an object of the 'HistogramMatching' class
		HM.histMatch(img1, img2);
		t = clock() - t;
		t_vec.push_back(t);
		//std::cout << "t_vec clicks: " << t_vec[iter] << " t_vec ms: " << (float)t_vec[iter] / CLOCKS_PER_SEC *1000
		//	<< " t clicks: " << t << " t ms: " << (float)t / CLOCKS_PER_SEC*1000 << std::endl;
	}
	float sum = std::accumulate(t_vec.begin(), t_vec.end(), 0)/t_vec.size();
	//std::cout << sum << " " << (float)sum/CLOCKS_PER_SEC*1000 << std::endl;
	std::cout << "Total time: " << (float)sum/CLOCKS_PER_SEC*1000 << "s " << "Avg. time per cycle: " << (float)sum / CLOCKS_PER_SEC * 1000 << "ms " << std::endl;
	cv::imshow("Img1", img1);
	cv::waitKey(0);
	return 0;
}
