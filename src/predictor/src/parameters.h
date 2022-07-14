// parameters.h
#pragma once
#ifndef parameters_H
#define parameters_H

//Standard Libraries
#include <vector>

//OpenCV Libraries
#include <opencv2/opencv.hpp>

class Parameters
{
public:
    double pi = 3.14159265358979323846; //Defined the value of pi

    //Parameters to change
    double added_delay2 = 1; //Delay in the camera feed

    //Hardware setup factors which are constants throughout the experiment

    //Global Variables
    double t0_PTZ, t0_omni; //Used in Delay Callback, shows the time at which the delayed PTZ frame was originally captured
    
    std::vector<double> T_array; //Array that stores the time and the corresponding gimbal angles.
    std::vector<double> P_array; //Array that stores the corresponding gimbal pitch angles.
    std::vector<double> Y_array; //Array that stores the corresponding gimbal yaw angles.
    std::vector<double> R_array; //Array that stores the corresponding gimbal roll angles.
    double pitch_new, pitch_old, yaw_new, yaw_old, roll_new, roll_old;
    double pitch_imu_cur; double pitch_imu_prev; double roll_imu_cur; double roll_imu_prev; double yaw_imu_cur; double yaw_imu_prev;
    double t_new, t_old;
    double t_imu_cur; double t_imu_prev;
    //Camera intrinsic matrix for PTZcam (Pre determined)
    cv::Mat Kp = (cv::Mat_<double>(3, 3) << 2120.49, 0, 981.1,
        0, 2119.06, 535.16, 0, 0, 1);
    //New camera intrinsic matrix for PTZcam after undistortion (Pre determined)
    cv::Mat Kp_new = (cv::Mat_<double>(3, 3) << 2137.686589057478, 0, 959.5,
 0, 2136.24499215188, 539.5, 0, 0, 1);
 
    //Distortion vector for PTZcam (Pre determined)
    cv::Mat Dp = (cv::Mat_<double>(1, 5) << -3.94952719e-1, -6.68450461e-1, 
    5.26731133e-4, 1.10059397e-3, 7.20450676);

    cv::Mat orig_show_omni, orig_show_PTZ; //Initializing empty matrix for PTZ image and Omnicam
    cv::Mat PTZ_delay, omni_delay; //DELAYED PTZ and omni image 
    cv::Mat H = (cv::Mat_<double>(3, 3) << 3.662595664683535, 0.05440616409171908, -1999.161621659138, -0.2770712432149974, 3.76913088044113, -2018.638427077817,
   -0.0001093606810511352, -8.214723739695882e-05, 0.9999999999999999); //Initializing the Omni to PTZ homography matrix

    cv::Mat H_delay;
    
    //double t_node0, t_node1; //The algorithm delay present in the whole calculation
    
    //Camera intrinsic matrix for Omnicam (Pre determined)
    cv::Mat Ko = (cv::Mat_<double>(3, 3) << 445.17984003885283, 0.0, 721.2172155119429, 0.0, 445.8816513187479, 721.9473150432297, 0.0, 0.0, 1.0);
    //Distortion vector for the omnicam (Used for undistorting an Omnicam image)
    cv::Mat Do = (cv::Mat_<double>(1, 4) << -0.017496068223756347, -0.003243014339251435, 0.000839223410670477, -0.0004219234265229322);
    
    /*
    //Virtual Distance along zo axis, mm
    //double Z = 3200;
    //PTZ to Omni distance vector, in Omnicam frame: "to", mm
    cv::Mat to = (cv::Mat_<double>(3, 1) << 12.7, 152, 267);
    double fp = 5; //PTZ focal length, mm
    double fo = 1; //Omni focal length, mm
    */
    double ros_rate = 10;
    double ros_duration = 1/ros_rate;
    bool stitch = 1; //Stitching image or split screen display
    
    //int counter = 1; //For debugging
    //double t_debug0, t_debug1;
    bool PD = 1; //WHether you are running the predictor or not
};


#endif
