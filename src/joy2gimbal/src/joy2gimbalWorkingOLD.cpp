//Custom libraraies
//#include "parameters.h"

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



double added_delay1 = 1.0; //sec, 1st communication delay from commanded quaternions to reaching gimbal
double added_delay2 = 1.0; //sec, 2nd communication delay from gimbal IMU obtained to IMU angles being displayed

double rate = 10;
double max_angular_velocity = 20; //deg/s
double min_angular_velocity = 0; //deg/s
double max_stick_position = 1;
double min_stick_position = 0;
bool isFirstData = 0; //Initial value
bool isGimbalTopicPublished = 0; //Initially no gimbal data is received
double pi = 3.14159265358979323846; //Value of pi

//Global vaiable to store the pose data
geometry_msgs::TwistStamped twist_msg_cmd, twist_msg_prev;

//Currently obtained quaternion command
geometry_msgs::PoseStamped quat_msg_cmd, delayed_imu_ang;

//Storing Yaw and pitch in radians
double prev_ang_rate_yaw, prev_ang_rate_pitch;

//Data for debugging
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
    double cmd_msg_rate_t2 = LinInterp(delayed_msg_rate, cmd_msg_rate, t2, delayed_msg_time, cmd_msg_time);
    
    //Simpson's rule formula                    
    double cmd_msg_delta = (cmd_msg_time - delayed_msg_time)/8 * (delayed_msg_rate + 3 * cmd_msg_rate_t1 + 3 * cmd_msg_rate_t2 + cmd_msg_rate);
    return cmd_msg_delta;
}

// Read incoming actual IMU quaternions after a delay (added_delay2)
void gimbalActAnglesCB(const geometry_msgs::PoseStamped& msg)
{
    delayed_imu_ang = msg;
    double begin = ros::Time::now().toSec();
    //std::cout << "Difference: " << std::setprecision(16) << (begin - msg.header.stamp.sec - msg.header.stamp.nsec * 1e-9 ) * 1000 << " ms" << std::endl << std::endl;
    
    
    //////////////////////??DEBUG////////////////
    //Quaternion to be converted to angles
        double qx = msg.pose.orientation.x;
        double qy = msg.pose.orientation.y;
        double qz = msg.pose.orientation.z;
        double qw = msg.pose.orientation.w;    
        
        //Roll (z-axis rotation of the camera)
        double sr_cp = 2 * (qw * qx + qy * qz);
        double cr_cp = 1 - 2 * (qx * qx + qy * qy);
        double roll = atan2(sr_cp, cr_cp); //rad, will remain constant
        
        //Yaw (y-axis rotation of the camera)
        double sy_cp = +2.0 * (qw * qz + qx * qy);
        double cy_cp = +1.0 - 2.0 * (qy * qy + qz * qz);
        double yaw = atan2(sy_cp, cy_cp); //rad, need to zero it out
        //Pitch (x-axis rotation of the camera)
        double sp = 2 * (qw * qy - qz * qx); //rad
        double pitch;
        if (abs(sp) >= 1)
            pitch = copysign(pi / 2, sp); //use 90 deg if out of range, rad
        else
            pitch = asin(sp); //rad
    
    //std::cout << cnt << " " << yaw*180/pi << " " << twist_msg_cmd.twist.angular.y * 180/pi << std::endl;
    cnt = cnt + 1;
    //std::cout << "RollA: " << roll*180/pi << " PitchA: " << pitch*180/pi << " YawA: " << yaw*180/pi << std::endl;
    //std::cout << "Roll: " << twist_msg_prev.twist.angular.z * 180/pi << " Pitch: " << twist_msg_prev.twist.angular.x * 180/pi << " Yaw: " << twist_msg_prev.twist.angular.y * 180/pi << std::endl <<std::endl;
    
    
    if(isGimbalTopicPublished == 0)
    {
        isGimbalTopicPublished = 1;
        //Current actual IMU angles
        //pose_msg_prev.header.seq = 0;
        twist_msg_prev.header.stamp = ros::Time::now();
        twist_msg_prev.header.frame_id = "current_actual_angles";
    
        //Quaternion to be converted to angles
        double q_x = msg.pose.orientation.x;
        double q_y = msg.pose.orientation.y;
        double q_z = msg.pose.orientation.z;
        double q_w = msg.pose.orientation.w;    
        
        //Roll (z-axis rotation of the camera)
        double sinr_cosp = 2 * (q_w * q_x + q_y * q_z);
        double cosr_cosp = 1 - 2 * (q_x * q_x + q_y * q_y);
        twist_msg_prev.twist.angular.z = atan2(sinr_cosp, cosr_cosp); //rad, will remain constant
        
        //Yaw (y-axis rotation of the camera)
        double siny_cosp = +2.0 * (q_w * q_z + q_x * q_y);
        double cosy_cosp = +1.0 - 2.0 * (q_y * q_y + q_z * q_z);
        twist_msg_prev.twist.angular.y = atan2(siny_cosp, cosy_cosp); //rad, need to zero it out
        //Pitch (x-axis rotation of the camera)
        double sinp = 2 * (q_w * q_y - q_z * q_x); //rad
        if (abs(sinp) >= 1)
            twist_msg_prev.twist.angular.x = copysign(pi / 2, sinp); //use 90 deg if out of range, rad
        else
            twist_msg_prev.twist.angular.x = asin(sinp); //rad
        
        prev_ang_rate_yaw = 0; prev_ang_rate_pitch = 0;
        //std::cout << "Roll: " << twist_msg_prev.twist.angular.z * 180/pi << " Pitch: " << twist_msg_prev.twist.angular.x * 180/pi << " Yaw: " << twist_msg_prev.twist.angular.y * 180/pi << std::endl << std::endl;
        //std::cout << msg << std::endl;
    }
}

//Read incoming joystick command integrate and publish to commanded angles topic
void gimbalCmdAnglesCb(const sensor_msgs::Joy& msg)
{
    //Need to convert this msg from quaternion to euler
    if(isGimbalTopicPublished == 1)
    {
        //std::cout << delayed_imu_ang.header.stamp << std::endl;
    
        //std::cout<< "working" << std::endl;
        //double begin = ros::Time::now().toSec();
        
        //Convert current joystick command to current angle rate command
        //pose_msg_cmd.header.seq = 1;
        twist_msg_cmd.header.stamp = msg.header.stamp;
        twist_msg_cmd.header.frame_id = "current_angle_command";
        //std::cout << std::setprecision(16) << twist_msg_cmd.header.stamp  << std::endl;
        
        //std::cout << "Difference: " << std::setprecision(16) << (twist_msg_cmd.header.stamp.sec + twist_msg_cmd.header.stamp.nsec * 1e-9 - begin) * 1000 << " ms" << std::endl << std::endl;	

        //Yaw rate of the conventional gimbal frame (z-axis is yaw), rad/s
        double cur_ang_rate_yaw = -(max_angular_velocity - min_angular_velocity) / (max_stick_position - min_stick_position) * msg.axes[3] * pi/180;
        //Pitch rate of the conventional gimbal frame (y-axis is the pitch), rad/s
        double cur_ang_rate_pitch = -(max_angular_velocity - min_angular_velocity) / (max_stick_position - min_stick_position) * msg.axes[4] * pi/180;
        
        //double delta_yaw = SimpsonIntegrate(twist_msg_prev.header.stamp.sec + twist_msg_prev.header.stamp.nsec * 1e-9, twist_msg_cmd.header.stamp.sec + twist_msg_cmd.header.stamp.nsec * 1e-9, prev_ang_rate_yaw, cur_ang_rate_yaw);
        //double delta_pitch = SimpsonIntegrate(twist_msg_prev.header.stamp.sec + twist_msg_prev.header.stamp.nsec * 1e-9, twist_msg_cmd.header.stamp.sec + twist_msg_cmd.header.stamp.nsec * 1e-9, prev_ang_rate_pitch, cur_ang_rate_pitch);
        
        //Using th = th_dot * dt to integrate, midpoint integration
        double delta_yaw = (cur_ang_rate_yaw + prev_ang_rate_yaw)/2 * (twist_msg_cmd.header.stamp.sec + twist_msg_cmd.header.stamp.nsec * 1e-9 - twist_msg_prev.header.stamp.sec - twist_msg_prev.header.stamp.nsec * 1e-9);
        double delta_pitch = (cur_ang_rate_pitch + prev_ang_rate_pitch)/2 * (twist_msg_cmd.header.stamp.sec + twist_msg_cmd.header.stamp.nsec * 1e-9 - twist_msg_prev.header.stamp.sec - twist_msg_prev.header.stamp.nsec * 1e-9);
        
        twist_msg_cmd.twist.angular.y = twist_msg_prev.twist.angular.y + delta_yaw; //Yaw, rad
        twist_msg_cmd.twist.angular.x = twist_msg_prev.twist.angular.x + delta_pitch; //Pitch, rad
        twist_msg_cmd.twist.angular.z = twist_msg_prev.twist.angular.z; //Roll, rad
            
        twist_msg_prev.header.stamp = twist_msg_cmd.header.stamp;
        twist_msg_prev.twist.angular.y = twist_msg_cmd.twist.angular.y;
        twist_msg_prev.twist.angular.x = twist_msg_cmd.twist.angular.x;
        twist_msg_prev.twist.angular.z = twist_msg_cmd.twist.angular.z;
        
        //Save cur_ang_rate to prev_ang_rate
        prev_ang_rate_yaw = cur_ang_rate_yaw;
        prev_ang_rate_pitch = cur_ang_rate_pitch;
        
        //std::cout << "Roll: " << twist_msg_prev.twist.angular.z * 180/pi << " Pitch: " << twist_msg_prev.twist.angular.x * 180/pi << " Yaw: " << twist_msg_prev.twist.angular.y * 180/pi << std::endl <<std::endl;
        //std::cout << quat_msg_cmd << std::endl;
        
        //double end = ros::Time::now().toSec();
        
        //std::cout << "Diff: " << (end-begin)*1000 << " ms"  << std::endl;        
    }
}

// Read incoming joystick command and publish to the desired gimbal position topic
void joy2gimbalTargetPosDelayCB(const geometry_msgs::TwistStamped& msg)
{
    double end = ros::Time::now().toSec();
    //Convert the current angles to quaternions to be sent to gimbal
    quat_msg_cmd.header.stamp = msg.header.stamp;
    quat_msg_cmd.header.frame_id = "quaternion_command";
    
    //std::cout << "Delayed" << std::endl;
    //std::cout << msg.twist.angular.x << " " << twist_msg_cmd.twist.angular.x << std::endl;
    //std::cout << "Difference: " << std::setprecision(16) << (end - (twist_msg_cmd.header.stamp.sec + twist_msg_cmd.header.stamp.nsec * 1e-9)) * 1000 << " ms" << std::endl << std::endl;	
    
    
    quat_msg_cmd.pose.orientation.x = sin(msg.twist.angular.z/2) * cos(msg.twist.angular.x/2) * cos(msg.twist.angular.y/2) - cos(msg.twist.angular.z/2) * sin(msg.twist.angular.x/2) * sin(msg.twist.angular.y/2);
    quat_msg_cmd.pose.orientation.y = cos(msg.twist.angular.z/2) * sin(msg.twist.angular.x/2) * cos(msg.twist.angular.y/2) + sin(msg.twist.angular.z/2) * cos(msg.twist.angular.x/2) * sin(msg.twist.angular.y/2);
    quat_msg_cmd.pose.orientation.z = cos(msg.twist.angular.z/2) * cos(msg.twist.angular.x/2) * sin(msg.twist.angular.y/2) - sin(msg.twist.angular.z/2) * sin(msg.twist.angular.x/2) * cos(msg.twist.angular.y/2);
    quat_msg_cmd.pose.orientation.w = cos(msg.twist.angular.z/2) * cos(msg.twist.angular.x/2) * cos(msg.twist.angular.y/2) + sin(msg.twist.angular.z/2) * sin(msg.twist.angular.x/2) * sin(msg.twist.angular.y/2);
    
    //double begin = ros::Time::now().toSec();
    //std::cout << "Difference: " << std::setprecision(16) << (begin - msg.header.stamp.sec - msg.header.stamp.nsec * 1e-9) * 1000 << " ms" << std::endl << std::endl;
    //cnt = cnt + 1; avg = avg + (begin - msg.header.stamp.sec - msg.header.stamp.nsec * 1e-9) * 1000;
    //std::cout << "Average: " << avg/cnt << std::endl;
}

int main(int argc, char** argv)
{
    std::cout << "joy2gimbal started" << std::endl;
 
    ros::init(argc,argv,"joy2gimbal_node");
    ros::NodeHandle node;
    
    ros::Rate loop_rate(rate);

    
    
    ros::Subscriber sub_joy = node.subscribe("/joy_orig", 1, &gimbalCmdAnglesCb);
    //ros::Subscriber sub_ang = node.subscribe("/gimbal_imu_angles", 1, &gimbalActAnglesCb);
    
    //To add delays between the command received at the gimbal vs the imu angle reaching the display (added_delay2)
    message_filters::Subscriber<geometry_msgs::PoseStamped> sub_delay_imu(node, "/gimbal_imu_angles", 1);
    message_filters::TimeSequencer<geometry_msgs::PoseStamped> seq_delay_imu(sub_delay_imu, ros::Duration(added_delay2), ros::Duration(1/rate), int(1 + (rate * added_delay2)));
    seq_delay_imu.registerCallback(&gimbalActAnglesCB);
    
    ros::Publisher pub_quat_cmd = node.advertise<geometry_msgs::PoseStamped> ("/gimbal_target_orientation", 1);
    
    ros::Publisher pub_imu_delay = node.advertise<geometry_msgs::PoseStamped> ("/delayed_imu_angles", 1);
        
    ros::Publisher pub_ang_cmd = node.advertise<geometry_msgs::TwistStamped> ("/commanded_angles", 1);
    
    //To add delays between the command send vs the command received at the gimbal (added_delay1)
    message_filters::Subscriber<geometry_msgs::TwistStamped> sub_delay_cmd(node, "/commanded_angles", 1);
    message_filters::TimeSequencer<geometry_msgs::TwistStamped> seq_delay_cmd(sub_delay_cmd, ros::Duration(added_delay1), ros::Duration(1/rate), int(1 + (rate * added_delay1)));
    seq_delay_cmd.registerCallback(&joy2gimbalTargetPosDelayCB);
    
    
    while(ros::ok())
	{
	    ros::spinOnce();
	    if(!quat_msg_cmd.header.stamp.sec == 0) 
	    {   
	        pub_quat_cmd.publish(quat_msg_cmd); //Publish the delayed quaternion message to the gimbal, quaternions
	    }
            if(!twist_msg_cmd.header.stamp.sec == 0)
            {	    
		pub_ang_cmd.publish(twist_msg_cmd); //Publish the currently commanded angles, rad
	    }
	    if(!delayed_imu_ang.header.stamp.sec == 0)
	    {	        
	        pub_imu_delay.publish(delayed_imu_ang); //Publish the delayed gimbal IMU angles
	    }
	    loop_rate.sleep();
	}

    return 0;
}
