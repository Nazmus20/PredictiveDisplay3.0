#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "ros_sbgc_driver_node.h"
#include <signal.h>

GimbalController* gc_ptr;
int control_timeout = 0;
int rate = 30; //Commented out Haseeb's code
//int rate = 100;
int cntr = 0;
bool m_angle_control_flag = true;


void signal_callback_handler(int signal)
{
    exit(signal);
}

// Read incoming desired speeds
void gimbalTargetSpeedCb(const geometry_msgs::TwistStampedConstPtr& msg)
{
    control_timeout = rate/2;
    //REP 103 - Roll, Pitch, Yaw
    double roll_s = msg->twist.angular.x;
    double pitch_s = msg->twist.angular.y;
    double yaw_s = msg->twist.angular.z;
    gc_ptr->setRPYspd(roll_s,pitch_s,yaw_s);
    m_angle_control_flag = false;
}

// Read incoming desired angles
void gimbalTargetAnglesCb(const geometry_msgs::PoseStampedConstPtr& msg)
{
    if (m_angle_control_flag == false)
    {
        return;
    }
    control_timeout = rate/2;
    tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);



    gc_ptr->setRPY(roll,pitch,yaw);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gimbal_control_node");
    
    ros::NodeHandle n;

    bool printAngles = false;
    std::string devPath = "dev/ttyUSB0";
    int baudRate = 115200;

    n.param<int>("rate", rate, 30);
    n.param<bool>("printAngles", printAngles, false);
    n.param<std::string>("devPath", devPath, "/dev/ttyUSB0");
    n.param<int>("baudRate", baudRate, 115200);

    GimbalLimits limits;
    n.param<double>("maxRoll_deg", limits.max_roll, 0);
    n.param<double>("minRoll_deg", limits.min_roll, 0);
    n.param<double>("maxPitch_deg", limits.max_pitch, 30);
    n.param<double>("minPitch_deg", limits.min_pitch, -90);
    n.param<double>("maxYaw_deg", limits.max_yaw, 90);
    n.param<double>("minYaw_deg", limits.min_yaw, -90);

    std::cout << "Params:\n" << "Rate:\t\t" << rate << std::endl;
    std::cout << "Print Angle:\t" << printAngles << std::endl;
    std::cout << "Baud Rate:\t" << baudRate << std::endl;
    std::cout << "Device Path:\t" << devPath << "\n\n";

    ros::Rate loop_rate(rate);
    
    ros::Subscriber sub_ang = n.subscribe("gimbal_target_orientation", 1, gimbalTargetAnglesCb);
    ros::Subscriber sub_spd = n.subscribe("gimbal_target_speed", 1, gimbalTargetSpeedCb);    

    signal(SIGINT, signal_callback_handler);
    
    GimbalController gc(devPath, baudRate, &n, limits);
    gc_ptr = &gc;

    while (!gc.connect())
    {
        //sleep(2); //Commented out Haseeb's code
        usleep(10000);
    }

    gc.requestData();
    usleep(10000);            
            //std::cout << "Doing the other  thing" << std::endl;
    
    while(ros::ok())
    {
        int timeout = 10;
        while (timeout > 0 && gc.processData() != 0)
        {            
            usleep(10000);
            //std::cout << "Doing the thing" << std::endl;
            gc.requestData();
            usleep(10000);
            timeout--;            
        }
        if (timeout == 0)
        {
            std::cout << "No response from gimbal, attempting reconnect..." << cntr << std::endl; //Added a counter
            cntr = cntr + 1; //Added a counter
            while (!gc.connect())
            {
                //sleep(2); //Commented out Haseeb's code
                usleep(10000); 
            }
            continue;
        }
        
        if (printAngles)
        {
            gc.printAngles();
        }

        gc.publishAngles();

        if (control_timeout <= 0)
        {
            gc.setRPYspd(0.0,0.0,0.0);
            control_timeout = 0;
            m_angle_control_flag = true;
        }
        else
        {
            control_timeout--;
        }        

        ros::spinOnce();    
        loop_rate.sleep();
    }
    return(0);
}   
