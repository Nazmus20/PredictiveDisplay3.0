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
    double gamma = (*P).fo / (*P).fp; //Constant scaling factor;
	
    //Rotation matrix from delayed PTZ to PTZ at current time
    Mat R = (Mat_<double>(3, 3) << cos(yaw1-yaw0), 0, -sin(yaw1-yaw0), 
    sin(yaw1-yaw0)*sin(pitch1-pitch0), cos(pitch1-pitch0), cos(yaw1-yaw0)*sin(pitch1-pitch0), 
    sin(yaw1-yaw0)*cos(pitch1-pitch0), -sin(pitch1-pitch0), cos(yaw1-yaw0)*cos(pitch1-pitch0));
    
    /*
    //Euclidean Homography matrix from Omni to PTZ for the current angles
    Mat top_nT = -(Mat_<double>(3, 3) << 0, 0, (*P).to.at<double>(0, 0),
    0, 0, (*P).to.at<double>(1, 0),
    0, 0, (*P).to.at<double>(2, 0));
        //Top * no^T vector, a 3x3 vector with top the distance 
        //from PTZ to Omni expressed in PTZ frame and no is the 
        //normal to the omni frame which is constant at [0, 0, 1]^T 
        //Minus sign instead of plus because using Tpo instead of Top
     */
    (*P).H_delay = (*P).Kp_new * R * (*P).Kp_new.inv(); 
//std::cout << (*P).H_delay << std::endl;
/*     
    //Projective Homography matrix from Omni to PTZ for current frame
    Mat Gpo1 = (*P).Kp * Hpo1 * (*P).Kp.inv();
    //Rotation matrix from Omni to PTZ at past time
    Mat Rpo0 = (Mat_<double>(3, 3) << cos(yaw0), 0, -sin(yaw0),
    sin(yaw0) * sin(pitch0), cos(pitch0), cos(yaw0) * sin(pitch0),
    sin(yaw0) * cos(pitch0), -sin(pitch0), cos(yaw0) * cos(pitch0));
    //Euclidean Homography matrix from Omni to PTZ for the past angles
    Mat Hpo0 = Rpo0 + (top_nT) / (*P).Z;

    //Projective Homography matrix from Omni to PTZ for past frame
    Mat Gpo0 = (*P).Kp * Hpo0 * (*P).Ko.inv();
*/
    ImageStitching IS2; //Create an instance of ImageStitching class

    IS2.alignImages((*P).omni_delay, (*P).PTZ_delay, (*P).H, (*P).H_delay, P);
    //std_msgs::Header h = (*P).PTZ_delay->header;
    //double t = double(h.stamp.sec) + double(h.stamp.nsec)*1e-9;
    
    
}

void HomographyCalculation::PoseCallback(const boost::shared_ptr<const geometry_msgs::PoseStamped>& msg, Parameters* P)
{
    
    //cout << "In QuatCB: " << (*P).orig_show_delay << endl;
    if (!(*P).PTZ_delay.empty() && !(*P).omni_delay.empty())
    {
    
    double begin = ros::Time::now().toSec();
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
          
        
            /*
        (*P).t_new = (*msg).header.stamp.sec + 1e-9 * (*msg).header.stamp.nsec;
        (*P).pitch_new = (*msg).twist.angular.x; 
        (*P).yaw_new = (*msg).twist.angular.y;
        (*P).roll_new = (*msg).twist.angular.z;
 		
        (*P).T_array.push_back((*P).t_new);
        (*P).P_array.push_back((*P).pitch_new);
        (*P).R_array.push_back((*P).roll_new);
        (*P).Y_array.push_back((*P).yaw_new);  
 
*/
        //At 30 FPS of the algorithm, we need to store 5 secs worth of data, so 150 data
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
        
        //std::cout << std::setprecision(16) << (*P).t0_PTZ << std::endl;
        //Compare the gimbal angles with the stored values to find the gimble angle of the displayed image
        vector<double> value_comp; //A vector that compares the absolute time difference between (*P).t0_PTZ and T_array elements 
        for (int i = 0; i < (*P).T_array.size(); i++)
        {
            value_comp.push_back(abs((*P).t0_PTZ - (*P).T_array[i]));
            
        }
        auto min_val = min_element(value_comp.begin(), value_comp.end()); //Find the index position of the minimum of "abs(tau0-t1)"
        int min_idx = int(distance(value_comp.begin(), min_val));
        //std::cout << "Idx: " << min_idx << std::endl;
        //std::cout << "Difference: " << ((*P).T_array[min_idx] - (*P).t0_PTZ)*1000 << std::endl;
        (*P).pitch_old = (*P).P_array[min_idx];
        (*P).yaw_old = (*P).Y_array[min_idx];
        
        //std::cout << std::setprecision(16) << (*P).t_imu_cur << " " << (*P).yaw_imu_cur << " " << (*P).t_new << " " << (*P).yaw_new << " " << (*P).t0_PTZ << " " << (*P).yaw_old << " " << ((*P).t_new - (*P).t0_PTZ) * 1000 << std::endl; 
        //std::cout << "Theta: " << ((*P).pitch_new - (*P).pitch_old) * 180/(*P).pi  << std::endl;
              
              
                 
        //std::cout << (*P).yaw_old << std::endl;
        HomographyCalculation::homCalc((*P).pitch_old, (*P).pitch_new, (*P).yaw_old, (*P).yaw_new, P);

    	

        (*P).t_node1 = ros::Time::now().toSec();
    	//double t_node = t_node1 - t_node0;
        //cout << "Alg time: " << setprecision(15) << (t1-t_node0)*1000 << endl;
	//cout << "YAW: " << yaw1*180/pi << " pitch: " << pitch1*180/pi << endl;
    }
}
