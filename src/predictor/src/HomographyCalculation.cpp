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

//Calculate homography based on gimbal IMU angles
void HomographyCalculation::homCalc(double pitch0, double pitch1, double yaw0, double yaw1, Parameters* P)
{    
    //Rotation matrix from delayed PTZ to PTZ at current time
    Mat R = (Mat_<double>(3, 3) << cos(yaw1-yaw0), 0, -sin(yaw1-yaw0), 
    sin(yaw1-yaw0)*sin(pitch1-pitch0), cos(pitch1-pitch0), cos(yaw1-yaw0)*sin(pitch1-pitch0), 
    sin(yaw1-yaw0)*cos(pitch1-pitch0), -sin(pitch1-pitch0), cos(yaw1-yaw0)*cos(pitch1-pitch0));
    
    (*P).H_delay = (*P).Kp_new * R * (*P).Kp_new.inv(); 
    ImageStitching IS2; //Create an instance of ImageStitching class

    IS2.alignImages((*P).omni_delay, (*P).PTZ_delay, (*P).H, (*P).H_delay, P);
}

void HomographyCalculation::PoseCallback(const boost::shared_ptr<const geometry_msgs::PoseStamped>& msg, Parameters* P)
{
    
    //cout << "In QuatCB: " << (*P).orig_show_delay << endl;
    if (!(*P).PTZ_delay.empty() && !(*P).omni_delay.empty())
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
          
        //At 10 FPS of the algorithm, we need to store 5 secs worth of data, so 50 data
        if ((*P).T_array.size() > 50)
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
        
        //Compare the gimbal angles with the stored values to find the gimble angle of the displayed image
        vector<double> value_comp; //A vector that compares the absolute time difference between (*P).t0_PTZ and T_array elements 
        for (int i = 0; i < (*P).T_array.size(); i++)
        {
            value_comp.push_back(abs((*P).t0_PTZ - (*P).T_array[i]));       
        }
        auto min_val = min_element(value_comp.begin(), value_comp.end()); //Find the index position of the minimum of "abs(tau0-t1)"
        int min_idx = int(distance(value_comp.begin(), min_val));
        (*P).pitch_old = (*P).P_array[min_idx];
        (*P).yaw_old = (*P).Y_array[min_idx];
        
        HomographyCalculation::homCalc((*P).pitch_old, (*P).pitch_new, (*P).yaw_old, (*P).yaw_new, P);
    }
}
