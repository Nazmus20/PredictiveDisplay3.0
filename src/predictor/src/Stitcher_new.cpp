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

#define pi 3.14159265358979323846 //Defined the value of pi
using namespace std;
using namespace cv;

//Parameters to change
double added_delay = .5;
double delay_time = added_delay;

//Hardware setup factors which are constants throughout the experiment

//Global Variables
double tau0; //Used in Delay Callback, shows the time at which the delayed frame was originally captured
vector<double> quatMsgT_array; //Array that stores the time and the corresponding gimbal angles.
vector<double> quatMsgP_array; //Array that stores the corresponding gimbal angles.
vector<double> quatMsgY_array; //Array that stores the corresponding gimbal angles.
double pitch_new, pitch_old, yaw_new, yaw_old;

//Camera intrinsic matrix for PTZcam (Pre determined)
Mat Kp = (Mat_<double>(3, 3) << 2233.1209,	0,	972.6046,
    0,	2226.9367,	629.569,
    0,	0,	1);
//New camera intrinsic matrix for PTZcam after undistortion (Pre determined)
cv::Mat Kp_new = (cv::Mat_<double>(3, 3) << 2.03229138e3, 0, 9.78737087e2,
    0, 2.01388062e3, 5.35125976e2,
    0, 0, 1);
//Distortion vector for PTZcam (Pre determined)
cv::Mat Dp = (cv::Mat_<double>(1, 5) << -.49561087, .32802248, 
    -.00887597, -.00106242, .93175615);
Mat orig_show_omni, orig_show_PTZ; //Initializing empty matrix for PTZ image and Omnicam
Mat orig_show_delay; //DELAYED PTZ image 
Mat H1, H_pred; //Initializing both the homography matrices
//double pitch1, yaw1; //Current Pitch and Yaw of the gibmal, rad (found in QuatCallback)
//double pitch0, yaw0; //Pitch and Yaw of the gibmal before the roundtrip delay, rad (found in QuatCallback)
double t_node0, t_node1; //The algorithm delay present in the whole calculation
//Camera intrinsic matrix for Omnicam (Pre determined)
Mat Ko = (Mat_<double>(3, 3) << 445.17984003885283, 0.0, 721.2172155119429, 0.0, 445.8816513187479, 721.9473150432297, 0.0, 0.0, 1.0);
//Distortion vector for the omnicam (Used for undistorting an Omnicam image)
Mat D = (Mat_<double>(1, 4) << -0.017496068223756347, -0.003243014339251435, 0.000839223410670477, -0.0004219234265229322);
//Virtual Distance along zo axis, mm
double Z = 3200;
//PTZ to Omni distance vector, in Omnicam frame: "to", mm
Mat to = (Mat_<double>(3, 1) << 12.7, 152, 267);
double fp = 5; //PTZ focal length, mm
double fo = 1; //Omni focal length, mm

Mat tvec = (Mat_<double>(3, 1) << 0, 0, 2000);
Mat rvec = (Mat_<double>(3, 1) << 0, 0, 0);
vector<Point3f> draw_lines;
vector<Point2f> image_pts;

//'alignImages' function called inside 'homCalc' function im1 is "delayed" Omnicam image im2 is "delayed" PTZ image
void alignImages(Mat& im1, Mat& im2, Mat h2, Mat h1)
{   
    if (!im1.empty() && !im2.empty())
    { 
        Mat h21 = h2 * h1.inv();
        // Use homography to warp image
    	Mat im1Reg;
    	warpPerspective(im1, im1Reg, h1, im2.size());
    	//imshow("imOmni to im1Omni", im1Reg);
    	Mat im2Reg;
    	warpPerspective(im1, im2Reg, h2, im2.size());
    	//imshow("imOmni to im5Omni", im2Reg);
    	Mat im21Omni, im21PTZ;
    	warpPerspective(im2, im21PTZ, h21, im2.size());
    	//imshow("im1PTZ to im5Omni", im21PTZ);
    	warpPerspective(im1Reg, im21Omni, h21, im2.size());
    	//imshow("im1Omni to im5Omni", im21Omni);
    	Mat dst_subtract;
    	subtract(im2Reg, im21Omni, dst_subtract);
    	//imshow("Subtracted image", dst_subtract);
    	//waitKey(1);
    	Mat ouImage;
    	add(im21PTZ, dst_subtract, ouImage);
    	// Resize image to have the same output resolution as the display screen
    	resize(ouImage, ouImage, Size(1215, 665), 0);
    	circle(ouImage, Point(608, 333), 5, Scalar(0, 0, 255), FILLED);
    	imshow("Final", ouImage);
    	waitKey(1);
    }
}

//Calculate homography based on gimbal IMU angles
void homCalc(double pitch0, double pitch1, double yaw0, double yaw1)
{    
    double gamma = fo / fp; //Constant scaling factor;

    //Rotation matrix from Omni to PTZ at current time
    Mat Rpo1 = (Mat_<double>(3, 3) << cos(yaw1), 0, -sin(yaw1), 
        sin(yaw1)*sin(pitch1), cos(pitch1), cos(yaw1)*sin(pitch1), 
        sin(yaw1)*cos(pitch1), -sin(pitch1), cos(yaw1)*cos(pitch1));
    //Euclidean Homography matrix from Omni to PTZ for the current angles
    Mat top_nT = -(Mat_<double>(3, 3) << 0, 0, to.at<double>(0, 0),
        0, 0, to.at<double>(1, 0),
        0, 0, to.at<double>(2, 0));
        //Top * no^T vector, a 3x3 vector with top the distance 
        //from PTZ to Omni expressed in PTZ frame and no is the 
        //normal to the omni frame which is constant at [0, 0, 1]^T 
        //Minus sign instead of plus because using Tpo instead of Top
    Mat Hpo1 = Rpo1 + (top_nT)/Z; 
    
    //Projective Homography matrix from Omni to PTZ for current frame
    Mat Gpo1 = Kp * Hpo1 * Ko.inv();

    //Rotation matrix from Omni to PTZ at past time
    Mat Rpo0 = (Mat_<double>(3, 3) << cos(yaw0), 0, -sin(yaw0),
        sin(yaw0) * sin(pitch0), cos(pitch0), cos(yaw0) * sin(pitch0),
        sin(yaw0) * cos(pitch0), -sin(pitch0), cos(yaw0) * cos(pitch0));
    //Euclidean Homography matrix from Omni to PTZ for the past angles
    Mat Hpo0 = Rpo0 + (top_nT) / Z;

    //Projective Homography matrix from Omni to PTZ for past frame
    Mat Gpo0 = Kp * Hpo0 * Ko.inv();
    alignImages(orig_show_omni, orig_show_delay, Gpo1, Gpo0);
}

void QuatCallback(const geometry_msgs::QuaternionStamped& msg) {
    t_node0 = ros::Time::now().toSec();
    if (!orig_show_delay.empty())
    { 
        //If the delayed image is open and displaying then perform all the following actions
        //getting the current time t1 defined globally
        double t1 = double(msg.header.stamp.sec) + double(msg.header.stamp.nsec)*1e-9; //Getting the timestamp of the current message
        ///////////// Quaternion to Euler ///////////////////////
        double q_x = msg.quaternion.x;
        double q_y = msg.quaternion.y;
        double q_z = msg.quaternion.z;
        double q_w = msg.quaternion.w;
        //Current Pitch and Yaw of the gibmal, rad
        // Roll (z-axis rotation) NOT NEEDED for PTZ
        //double sinr_cosp = +2.0 * (q_w * q_x + q_y * q_z);
        //double cosr_cosp = +1.0 - 2.0 * (q_x * q_x + q_y * q_y);
        //double pitch1 = atan2(sinr_cosp, cosr_cosp);
        
        // yaw (y-axis rotation)
        double siny_cosp = +2.0 * (q_w * q_z + q_x * q_y);
        double cosy_cosp = +1.0 - 2.0 * (q_y * q_y + q_z * q_z);
        double yaw1 = -atan2(siny_cosp, cosy_cosp);
	//Pitch (x-axis rotation)
	double sinp = 2* (q_w * q_y - q_z * q_x);
	double pitch1;
	if (abs(sinp) >= 1)
	    pitch1 = -copysign(pi / 2, sinp); //use 90 deg if out of range
	else
	    pitch1 = -asin(sinp);        

        quatMsgT_array.push_back(t1);
        quatMsgP_array.push_back(pitch1);
        quatMsgY_array.push_back(yaw1);
        if (quatMsgT_array.size() > int(100)) //Store only the latest "int(16*delay_time)" values
        {
            quatMsgT_array.erase(quatMsgT_array.begin());
            quatMsgP_array.erase(quatMsgP_array.begin());
            quatMsgY_array.erase(quatMsgY_array.begin());
        }
        //Pitch and Yaw of the gibmal before the roundtrip delay tau, rad
        double pitch0, yaw0;
        if (tau0 < quatMsgT_array[0])
        {
            pitch0 = 0; //quatMsgP_array[0];
            yaw0 = 0; //quatMsgY_array[0];
        }
        else
        { 
            vector<double> value_comp; //A vector that compares the absolute time difference between t1 and tau0 
            for (int i = 0; i < quatMsgT_array.size(); i++)
            {
                value_comp.push_back(abs(tau0 - quatMsgT_array[i]));
            }
            auto min_val = min_element(value_comp.begin(), value_comp.end()); //Find the index position of the minimum of "abs(tau0-t1)"
            int min_idx = int(distance(value_comp.begin(), min_val));
            pitch0 = quatMsgP_array[min_idx];
            yaw0 = quatMsgY_array[min_idx];
        }
	
        homCalc(pitch0, pitch1, yaw0, yaw1);

        t_node1 = ros::Time::now().toSec();
    	//double t_node = t_node1 - t_node0;
        //cout << "Alg time: " << setprecision(15) << (t1-t_node0)*1000 << endl;
	    //cout << "YAW: " << yaw1*180/pi << " pitch: " << pitch1*180/pi << endl;
    }
}


void imageCb_ptz(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    //Get time header info
    std_msgs::Header h = msg->header;
    tau0 = double(h.stamp.sec) + double(h.stamp.nsec)*1e-9;
    double tau1 = ros::Time::now().toSec();//chrono::system_clock::to_time_t(chrono::system_clock::now());
    double tau = tau1 - tau0;
    //cout << "Total delay: " << setprecision(15) << tau*1000 << " ms" << endl;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat src_ptz, src_PTZ;
    src_ptz = cv_ptr->image;
	resize(src_ptz, src_PTZ, Size(1920, 1080), 0);
    // To show image
    Mat orig_show;
    //undistort(src_ptz, orig_show, Kp, Dp, Kp_new);
    //resize(orig_show, orig_show_PTZ, Size(1920, 1080), 0);
    circle(src_PTZ, Point(960, 540), 5, Scalar(0, 0, 255), FILLED);
    int x = int(1106/2);
    int y = int(692/2);
    Point P1 = Point(960-579, 540-326);
    Point P2 = Point(960+518, 540-311);
    Point P3 = Point(960-578, 540+317);
    Point P4 = Point(960+511, 540+304);
    //Top-left corner
    circle(src_PTZ, P1, 5, Scalar(0, 255, 0), FILLED);
    //Top-right corner
    circle(src_PTZ, P2, 5, Scalar(0, 255, 0), FILLED);
    //Bot-left corner
    circle(src_PTZ, P3, 5, Scalar(0, 255, 0), FILLED);    
    //Bot-right corner
    circle(src_PTZ, P4, 5, Scalar(0, 255, 0), FILLED);
	
    //Top line
    line(src_PTZ, P1, P3, Scalar(0, 255, 0), 2, LINE_8);
    //Right line
    line(src_PTZ, P3, P4, Scalar(0, 255, 0), 2, LINE_8);
    //Bot line 
    line(src_PTZ, P4, P2, Scalar(0, 255, 0), 2, LINE_8);
    //Left line 
    line(src_PTZ, P2, P1, Scalar(0, 255, 0), 2, LINE_8);
        
    namedWindow("image_ptz", WND_PROP_FULLSCREEN);
    setWindowProperty("image_ptz", WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN); 
    //imshow("image_ptz", orig_show_PTZ);
    imshow("image_ptz", src_PTZ);
    
    cv::waitKey(1); 

}

void delayCb(const boost::shared_ptr<sensor_msgs::Image>msg){
	    
    cv_bridge::CvImagePtr cv_ptr;
   //Get time header info
    std_msgs::Header h = msg->header;
    tau0 = double(h.stamp.sec) + double(h.stamp.nsec)*1e-9;
    double tau1 = ros::Time::now().toSec();//chrono::system_clock::to_time_t(chrono::system_clock::now());
    double tau = tau1 - tau0;
    //cout << "Total delay: " << setprecision(15) << tau*1000 << " ms" << endl;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

	Mat src_ptz;
    	src_ptz = cv_ptr->image;

	// To flip image
    flip(src_ptz, orig_show_delay, -1);
	//imshow("Delayed image_ptz",orig_show_delay);
	//cv::waitKey(1);
}

int main(int argc, char** argv){

    ros::init(argc,argv,"image_prediction");
    ros::NodeHandle node;

    // Normal ROS subscriber

    ros::Subscriber sub = node.subscribe("/unnamed/control/camera_orientation", 1, QuatCallback);
    // Image subscriber should use different node name and subscriber.
    image_transport::ImageTransport it_(node);
    //image_transport::TransportHints hints("compressed", ros::TransportHints()); // To interpret compressed video not required here as the videos are already uncompressed
    //image_transport::Subscriber image_sub_omni = it_.subscribe("/omnicam/omnicam/image_raw", 1, imageCb_omni);
    image_transport::Subscriber image_sub_ptz = it_.subscribe("/PTZcam/PTZcam/image_raw", 1, &imageCb_ptz);
    
    //To add delays to pics
    //message_filters::Subscriber<sensor_msgs::Image> sub_delay_ptz(node, "/PTZcam/PTZcam/image_raw", 1);
    //message_filters::TimeSequencer<sensor_msgs::Image> seq_delay_ptz(sub_delay_ptz, ros::Duration(added_delay), ros::Duration(.066), int(15*delay_time));
    //seq_delay_ptz.registerCallback(delayCb);
    ros::Rate loop_rate(30);
	while(ros::ok())
	{
		ros::spinOnce();
        	loop_rate.sleep();
	}

	return 0;
}
