// HomographyCalculation.cpp

//Include class
#include "HomographyCalculation.h"
#include "ImageStitching.h"
//#include "HistogramMatching.h"

using namespace std;
using namespace cv;

HomographyCalculation::HomographyCalculation() //Default constructor
{
}

HomographyCalculation::~HomographyCalculation() //Default destructor
{
}

//Binary Search
vector<int> HomographyCalculation::BinarySearch(vector<double> &arr, double &num)
{
    vector<int> idx;
    int lo = 0; int hi = arr.size() - 1; int mid;
    while(hi-lo > 1)
    {
        mid = int((hi+lo)/2);
        if(arr[mid] < num)
        {
            lo = mid + 1;
        }
        else
        {
            hi = mid;
        }
    }
    if(arr[lo] == num)
    {
        idx.push_back(lo); idx.push_back(lo);
        return idx;
    }
    else if(arr[hi] == num)
    {
        idx.push_back(hi); idx.push_back(hi);
        return idx;
    }
    else
    {
        idx.push_back(lo); idx.push_back(hi);
        return idx;
    }    
}

//Linear interpolation
double HomographyCalculation::LinInterp(double y_start, double y_final, double x, double x_start, double x_final)
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

//Calculate homography based on gimbal IMU angles
void HomographyCalculation::homCalc(double pitch_ptz, double pitch_omni, double yaw_ptz, double yaw_omni, Parameters* P)
{    
    //Rotation matrix from delayed PTZ to PTZ at current time
    Mat R_ptz2ptz = (Mat_<double>(3, 3) << cos((*P).yaw_new-yaw_ptz), 0, -sin((*P).yaw_new-yaw_ptz), 
    sin((*P).yaw_new-yaw_ptz)*sin((*P).pitch_new-pitch_ptz), cos((*P).pitch_new-pitch_ptz), cos((*P).yaw_new-yaw_ptz)*sin((*P).pitch_new-pitch_ptz), 
    sin((*P).yaw_new-yaw_ptz)*cos((*P).pitch_new-pitch_ptz), -sin((*P).pitch_new-pitch_ptz), cos((*P).yaw_new-yaw_ptz)*cos((*P).pitch_new-pitch_ptz));
    
    //Rotation matrix from delayed omni to delayed PTZ
    Mat R_omni2ptz = (Mat_<double>(3, 3) << cos(yaw_ptz-yaw_omni), 0, -sin(yaw_ptz-yaw_omni), 
    sin(yaw_ptz-yaw_omni)*sin(pitch_ptz-pitch_omni), cos(pitch_ptz-pitch_omni), cos(yaw_ptz-yaw_omni)*sin(pitch_ptz-pitch_omni), 
    sin(yaw_ptz-yaw_omni)*cos(pitch_ptz-pitch_omni), -sin(pitch_ptz-pitch_omni), cos(yaw_ptz-yaw_omni)*cos(pitch_ptz-pitch_omni));
    
    
    Mat H_ptz2ptz_delay = (*P).Kp_new * R_ptz2ptz * (*P).Kp_new.inv(); 
    Mat H_omni2ptz_delay = (*P).Kp_new * R_omni2ptz * (*P).Kp_new.inv();
    
    ImageStitching IS2; //Create an instance of ImageStitching class

    //IS2.alignImages((*P).omni_delay, (*P).ptz_delay, (*P).H, H_ptz_ptz_delay, P); //OLD
    IS2.alignImages((*P).omni_delay, (*P).ptz_delay, H_omni2ptz_delay, H_ptz2ptz_delay, P); 
}

//Convert the actual gimbal quaternions to actual poses
void HomographyCalculation::PoseCallback(const boost::shared_ptr<const geometry_msgs::PoseStamped>& msg, Parameters* P)
{
    if (!(*P).ptz_delay.empty() && !(*P).omni_delay.empty())
    {
        //Delayed gimbal IMU angles and their timestamps
        (*P).t_imu_cur = (*msg).header.stamp.sec + 1e-9 * (*msg).header.stamp.nsec;

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
          
        //At 30 FPS of the algorithm, we need to store 10 secs worth of data, so 300 data
        if ((*P).T_array.size() > 300)
        {
            (*P).T_array.erase((*P).T_array.begin());
            (*P).R_array.erase((*P).R_array.begin());
            (*P).P_array.erase((*P).P_array.begin());
            (*P).Y_array.erase((*P).Y_array.begin());
        }
        
        (*P).T_array.push_back((*P).t_imu_cur);
        (*P).P_array.push_back((*P).pitch_imu_cur);
        (*P).R_array.push_back((*P).roll_imu_cur);
        (*P).Y_array.push_back((*P).yaw_imu_cur); 
        
        
    
        //If camera is sending data faster than the gimbal ... wait!!!
        if((*P).T_array.size() >= 1)
        {
            //Compare the gimbal angles with the stored values to find the gimble angle of the displayed images
            double pitch_old_ptz, yaw_old_ptz, pitch_old_omni, yaw_old_omni;

            //For PTZcam
            if((*P).t0_PTZ >= (*P).T_array.back()) //If gimbal lags behind camera, the image will move with gimbal
            {  
                pitch_old_ptz = (*P).P_array.back();
                yaw_old_ptz = (*P).Y_array.back();
            }
            else if((*P).t0_PTZ < (*P).T_array[0]) //If camera is so laggy that it crosses the storage threshold --> Increase storage
            {
                std::cout << "PTZcam delay too large. Increase array size" << std::endl;
            }
            else //Otherwise, do Binary search through the array to find matches
            {
                vector<int> ptz_idx = HomographyCalculation::BinarySearch((*P).T_array, (*P).t0_PTZ);
                pitch_old_ptz = HomographyCalculation::LinInterp((*P).P_array[ptz_idx[1]], (*P).P_array[ptz_idx[2]], (*P).t0_PTZ, (*P).T_array[ptz_idx[1]], (*P).T_array[ptz_idx[2]]);
                yaw_old_ptz = HomographyCalculation::LinInterp((*P).Y_array[ptz_idx[1]], (*P).Y_array[ptz_idx[2]], (*P).t0_PTZ, (*P).T_array[ptz_idx[1]], (*P).T_array[ptz_idx[2]]);
            }
    
            if((*P).t0_omni >= (*P).T_array.back()) //If gimbal lags behind camera, the image will move with gimbal
            {
                pitch_old_omni = (*P).P_array.back();
                yaw_old_omni = (*P).Y_array.back();
            }
            else if((*P).t0_omni < (*P).T_array[0]) //If camera is so laggy that it crosses the storage threshold --> Increase storage
            {
                std::cout << "Omnicam delay too large. Increase array size" << std::endl;
            }
            else //Otherwise, do Binary search through the array to find matches
            {
                vector<int> omni_idx = HomographyCalculation::BinarySearch((*P).T_array, (*P).t0_PTZ);
                pitch_old_omni = HomographyCalculation::LinInterp((*P).P_array[omni_idx[1]], (*P).P_array[omni_idx[2]], (*P).t0_omni, (*P).T_array[omni_idx[1]], (*P).T_array[omni_idx[2]]);
                yaw_old_omni = HomographyCalculation::LinInterp((*P).Y_array[omni_idx[1]], (*P).Y_array[omni_idx[2]], (*P).t0_omni, (*P).T_array[omni_idx[1]], (*P).T_array[omni_idx[2]]);
            }
        
            
            
        /*
        ////////////OLD////////////////////
        //Compare the gimbal angles with the stored values to find the gimble angle of the displayed images
        vector<double> value_comp; //A vector that compares the absolute time difference between (*P).t0_PTZ and T_array elements 
        for (int i = 0; i < (*P).T_array.size(); i++)
        {
            value_comp.push_back(abs((*P).t0_PTZ - (*P).T_array[i]));       
        }
        auto min_val = min_element(value_comp.begin(), value_comp.end()); //Find the index position of the minimum of "abs(tau0-t1)"
        int min_idx = int(distance(value_comp.begin(), min_val));
        (*P).pitch_old = (*P).P_array[min_idx];
        (*P).yaw_old = (*P).Y_array[min_idx];
        ////////////////////////////////////////////////
        */
        
        HomographyCalculation::homCalc(pitch_old_ptz, pitch_old_omni, yaw_old_ptz, yaw_old_omni, P);
        }
    }        
}
