
//Standard Libraries
//#include <iostream>
//#include <fstream>
//#include <ctime>
//#include <cmath>
//#include "bits/time.h"
//#include <stdio.h>
//#include <chrono>
//#include <numeric>
//#include <algorithm>
//#include <vector>

//OpenCV Libraries
//#include <opencv2/opencv.hpp>
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
//#include <opencv2/core/cuda.hpp>
//#include <opencv2/cudaarithm.hpp>
//#include <opencv2/cudaimgproc.hpp>

//ROS Libraries
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/TwistStamped.h>
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

int rate = 999;
int T = 30; //Total time of the sweep, s
double A = 10; //Amplitude of the input, deg/s
int Fs = rate; //Sampling frequency/ROS rate, Hz
double Ts = 1.0/Fs; //Sampling time, s
int N = int(T/Ts); //Number of samples
double w1 = 2*3.14159265359*.01; //Starting frequency, rad/s
double w2 = 2*3.14159265359*2; //Ending frequency, rad/s
double w_curr = w1; //Instantaneous frequency, rad/s
double dw = (w2-w1)/N/2; //frequency increment, rad/s
std::vector<double> w, sinw, t; //Frequency sweep vector, rad/s

int main(int argc, char** argv)
{
    ros::init(argc,argv,"freqSweep_node");
    ros::NodeHandle node;
    //ros::Publisher pub = node.advertise<geometry_msgs::TwistStamped> ("/gimbal_target_speed", 1);
    
    ros::Rate loop_rate(rate);
    
    geometry_msgs::TwistStamped twist_msg;
    
    for(int i=0; i<N; i++)
    {
        w.push_back(w1 + i*dw);
        t.push_back(i*Ts); 
        sinw.push_back(A*sin(w.at(i)*t.at(i)));
                   
    }
    
    //ros::Subscriber sub_ang = n.subscribe("gimbal_target_orientation", 1, gimbalTargetAnglesCb);
    //ros::Subscriber sub_spd = node.subscribe<sensor_msgs::Joy> ("/joy_orig", 1, boost::bind(joy2gimbalTargetSpeedCB, _1, &twist_msg));  
    ros::Publisher pub = node.advertise<geometry_msgs::TwistStamped> ("/gimbal_target_speed", 1);
    int I = 0; 
    
    double begin = ros::Time::now().toSec();
    
    while(ros::ok())
	{
	    ros::spinOnce();
	    double curr_time = ros::Time::now().toSec();
	    if (curr_time - begin >= 5)
	    {
	        if(I>=sinw.size())
	    	{
    	            std::cout << "Break" << std::endl;
	            double end = ros::Time::now().toSec();
	            double run_time = end - begin;
	            std::cout << std::setprecision(16) << run_time << std::endl;
	            break;
    	    	}
    	    	twist_msg.header.stamp = ros::Time::now();
    	    	twist_msg.header.frame_id = "freqSweep";
	    	//twist_msg.twist.angular.z = sinw.at(I); //For yaw
            	twist_msg.twist.angular.y = sinw.at(I); //For pitch
            	pub.publish(twist_msg); //Publish the message
   		I++;

    	    }
    	    else
    	    {
    	    	twist_msg.header.stamp = ros::Time::now();
    	    	twist_msg.header.frame_id = "freqSweep";
	    	twist_msg.twist.angular.z = 0; //For yaw
            	//twist_msg.twist.angular.y = 0; //For pitch
            	pub.publish(twist_msg); //Publish the message
    	    }
            loop_rate.sleep();
        }
    return 0;
}
