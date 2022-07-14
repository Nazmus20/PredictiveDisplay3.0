
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
#include <geometry_msgs/PoseStamped.h>
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
#include "message_filters/time_sequencer.h"
#include "message_filters/subscriber.h"
//#include <std_msgs/String.h>

double added_delay = 1.0; //sec, Joystick input delay
double rate = 999;
double max_angular_velocity = 20; //deg/s
double min_angular_velocity = 0; //deg/s
double max_stick_position = 1;
double min_stick_position = 0;
bool isFirstData = 0; //Initial value
bool isGimbalTopicPublished = 0; //Initially no gimbal data is received
double pi = 3.14159265358979323846; //Value of pi

//Global vaiable to store the pose data
geometry_msgs::TwistStamped pose_msg_cmd, pose_msg_delay;

//Rate command at the delayed gimbal IMU angle
geometry_msgs::TwistStamped twist_msg_prev;

//Data for debugginf
int cnt = 0; 
double avg = 0;

//Linear interpolation
double LinInterp(double y_start, double y_final, double x, double x_start, double x_final)
{
    //Slope
    double m = (y_final - y_start) / (x_final - x_start);
    //Interpolate 1
    double y1 = y_start + m * (x - x_start);
    //Interpolate 2
    double y2 = y_final - m * (x_final - x);
    //Average
    double y = (y1 + y2) / 2;
    
    return y;
}

//Integrate using 3/8 Simpson's rule
double SimpsonIntegrate(double delayed_msg_time, double cmd_msg_time, double delayed_msg_rate, double cmd_msg_rate)
{
    double t1 = (2*delayed_msg_time + cmd_msg_time)/3;
    double t2 = (delayed_msg_time + 2*cmd_msg_time)/3;
    
    //Linear interpolation of the rate based on t1 and t2
    double cmd_msg_rate_t1 = LinInterp(delayed_msg_rate, cmd_msg_rate, t1, delayed_msg_time, cmd_msg_time);
    double cmd_msg_rate_t2 = LinInterp(delayed_msg_rate, cmd_msg_rate, t1, delayed_msg_time, cmd_msg_time);
    
    //Simpson's rule formula                    
    double cmd_msg_delta = (cmd_msg_time - delayed_msg_time)/8 * (delayed_msg_rate + 3 * cmd_msg_rate_t1 + 3 * cmd_msg_rate_t2 + cmd_msg_rate);
    return cmd_msg_delta;
}

// Read incoming actual IMU angles
void gimbalActAnglesCb(const geometry_msgs::PoseStamped& msg)
{
    if(isGimbalTopicPublished == 0)
    {
        //Initially set the previous commanded angle rate to be 0, (static, no motion)
        twist_msg_prev.header.seq = msg.header.seq;
        twist_msg_prev.header.stamp = ros::Time::now();
        twist_msg_prev.header.frame_id = "previous_commanded_rate";
        twist_msg_prev.twist.angular.z = 0;
        twist_msg_prev.twist.angular.y = 0;
    }   
    else
    {
        //Set the current rate as the previous rate for next step in "gimbalCmdAnglesCB" 
        twist_msg_prev.header.seq = msg.header.seq;
        twist_msg_prev.header.stamp = ros::Time::now();
        twist_msg_prev.header.frame_id = "previous_commanded_rate";
    }
    
    isGimbalTopicPublished = 1;
    //Delayed actual IMU angles
    pose_msg_delay.header.seq = msg.header.seq;
    pose_msg_delay.header.stamp.sec = msg.header.stamp.sec;
    pose_msg_delay.header.stamp.nsec = msg.header.stamp.nsec;
    pose_msg_delay.header.frame_id = "delayed_actual_angles";
    
    // Quaternion to Euler
    double q_x = msg.pose.orientation.x;
    double q_y = msg.pose.orientation.y;
    double q_z = msg.pose.orientation.z;
    double q_w = msg.pose.orientation.w;
    
    //Yaw (y-axis rotation of the camera)
    double siny_cosp = +2.0 * (q_w * q_z + q_x * q_y);
    double cosy_cosp = +1.0 - 2.0 * (q_y * q_y + q_z * q_z);
    pose_msg_delay.twist.angular.y = -atan2(siny_cosp, cosy_cosp);;
    //Pitch (x-axis rotation of the camera)
    double sinp = 2 * (q_w * q_y - q_z * q_x);
    if (abs(sinp) >= 1)
        pose_msg_delay.twist.angular.x = -copysign(pi / 2, sinp); //use 90 deg if out of range
    else
        pose_msg_delay.twist.angular.x = -asin(sinp);
        
    //std::cout << ros::Time::now() << std::endl;
}

// Read incoming joystick command integrate and publish to commanded angles topic
void gimbalCmdAnglesCb(const geometry_msgs::TwistStamped& msg)
{
    //need to convert this msg from quaternion to euler
    if(isGimbalTopicPublished == 1)
    {
        if(isFirstData == 0)
        {
            isFirstData = 1;
            //Current commanded data will also be the same (no motion initially)
            pose_msg_cmd.header.seq = 0;
            pose_msg_cmd.header.stamp.sec = pose_msg_delay.header.stamp.sec;
            pose_msg_cmd.header.stamp.nsec = pose_msg_delay.header.stamp.nsec;
            pose_msg_cmd.header.frame_id = "Commanded_angles";
            //Yaw (y-axis rotation of the camera)
            pose_msg_cmd.twist.angular.y = pose_msg_delay.twist.angular.y;
            //Pitch (x-axis rotation of the camera)
            pose_msg_cmd.twist.angular.x = pose_msg_delay.twist.angular.x;
        }
        
        else
        {
            //Integrate the commanded angle rate to get the angle commanded using 3/8 Simpson's rule
            pose_msg_cmd.header.seq = pose_msg_cmd.header.seq + 1;
            pose_msg_cmd.header.stamp.sec = msg.header.stamp.sec;
            pose_msg_cmd.header.stamp.nsec = msg.header.stamp.nsec;
            pose_msg_cmd.header.frame_id = "Commanded_angles";
            //Yaw(y-axis rotation of the camera, z-axis in the incoming msg)
            pose_msg_cmd.twist.angular.y = pose_msg_delay.twist.angular.y + SimpsonIntegrate(twist_msg_prev.header.stamp.sec + 1e-9 * twist_msg_prev.header.stamp.nsec, msg.header.stamp.sec + 1e-9 * msg.header.stamp.nsec, twist_msg_prev.twist.angular.z, msg.twist.angular.z);
                    
            //Pitch (x-axis rotation of the camera, y-axis in the incoming msg)
            pose_msg_cmd.twist.angular.x = pose_msg_delay.twist.angular.x + SimpsonIntegrate(twist_msg_prev.header.stamp.sec + 1e-9 * twist_msg_prev.header.stamp.nsec, msg.header.stamp.sec + 1e-9 * msg.header.stamp.nsec, twist_msg_prev.twist.angular.y, msg.twist.angular.y); 
        }
        
        //Set the current commanded rate as the previous commanded rate
        if(twist_msg_prev.header.stamp.sec + 1e-9 * twist_msg_prev.header.stamp.nsec >= msg.header.stamp.sec + 1e-9 * msg.header.stamp.nsec)
        {
            twist_msg_prev.twist.angular.z = msg.twist.angular.z;
            twist_msg_prev.twist.angular.y = msg.twist.angular.y;   
            std::cout<< "Updating twist_msg_prev" << std::endl;         
        } 
    }
}

// Read incoming joystick command and publish to the desired gimbal topic
void joy2gimbalTargetSpeedDelayCB(const boost::shared_ptr<const sensor_msgs::Joy>& msg, geometry_msgs::TwistStamped* twist_msg)
{
    (*twist_msg).header.seq = (*msg).header.seq;
    //(*twist_msg).header.stamp = ros::Time::now();
    (*twist_msg).header.stamp.sec = (*msg).header.stamp.sec;
    (*twist_msg).header.stamp.nsec = (*msg).header.stamp.nsec;
    (*twist_msg).header.frame_id = "joy2gimbalDelay";
    //Yaw rate of the conventional aircraft frame (z-axis is yaw)
    (*twist_msg).twist.angular.z = -(max_angular_velocity - min_angular_velocity) / (max_stick_position - min_stick_position) * (*msg).axes[3];
    //Pitch rate of the conventional aircraft frame (x-axis is the pitch)
    (*twist_msg).twist.angular.y = (max_angular_velocity - min_angular_velocity) / (max_stick_position - min_stick_position) * (*msg).axes[4];
    
    //double begin = ros::Time::now().toSec();
    
    //std::cout << "Time now: " << begin << std::endl;
    //std::cout << "Header time: " << std::setprecision(16) << (*msg).header.stamp.sec + (*msg).header.stamp.nsec * 1e-9 << std::endl << std::endl;
    
    //cnt = cnt + 1; avg = avg + (begin - (*msg).header.stamp.sec - (*msg).header.stamp.nsec * 1e-9) * 1000;
    
    //std::cout << "Average: " << avg/cnt << std::endl;
    //std::cout << "Difference: " << std::setprecision(16) << (begin - (*msg).header.stamp.sec - (*msg).header.stamp.nsec * 1e-9) * 1000 << " ms" << std::endl << std::endl;
    
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"joy2gimbal_node");
    ros::NodeHandle node;
    
    ros::Rate loop_rate(rate);
    geometry_msgs::TwistStamped twist_msg;
    
    ros::Subscriber sub_ang_rate = node.subscribe("/gimbal_target_speed", 1, &gimbalCmdAnglesCb);
    ros::Subscriber sub_ang = node.subscribe("/gimbal_imu_angles", 1, &gimbalActAnglesCb);
    
    //ros::Subscriber sub_spd = node.subscribe<sensor_msgs::Joy> ("/joy_orig", 1, boost::bind(joy2gimbalTargetSpeedCB, _1, &twist_msg));  
    
    std::cout << "joy2gimbal started" << std::endl;
    
    //To add delays to gimbal node
    message_filters::Subscriber<sensor_msgs::Joy> sub_delay_joy(node, "/joy_orig", 1);
    message_filters::TimeSequencer<sensor_msgs::Joy> seq_delay_joy(sub_delay_joy, ros::Duration(added_delay), ros::Duration(1/rate), int(rate * added_delay));
    seq_delay_joy.registerCallback(boost::bind(&joy2gimbalTargetSpeedDelayCB, _1, &twist_msg));
    
    ros::Publisher pub_spd_cmd = node.advertise<geometry_msgs::TwistStamped> ("/gimbal_target_speed", 1);
        
    ros::Publisher pub_ang_cmd = node.advertise<geometry_msgs::PoseStamped> ("/commanded_angles", 1);

    while(ros::ok())
	{
	    ros::spinOnce();
	    pub_spd_cmd.publish(twist_msg);
  	    pub_ang_cmd.publish(pose_msg_cmd);
  	    
            loop_rate.sleep();
	}

    return 0;
}
