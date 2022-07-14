#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <SBGC_ComObj_Imp.h>
#include <SBGC_lib/SBGC_cmd_helpers.cpp>
#include <vector>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"

struct GimbalLimits
{
    double max_roll;
    double min_roll;
    double max_pitch;
    double min_pitch;
    double max_yaw;
    double min_yaw;
};

int getBaud(int baud)
{
    switch (baud) 
    {
        case 9600:
            return B9600;
        case 19200:
            return B19200;
        case 38400:
            return B38400;
        case 57600:
            return B57600;
        case 115200:
            return B115200;
        case 230400:
            return B230400;
        case 460800:
            return B460800;
        case 500000:
            return B500000;
        case 576000:
            return B576000;
        case 921600:
            return B921600;
        default: 
            return B115200;
    }
};


class GimbalController
{
public:
    GimbalController(std::string dev_path, int baudrate, ros::NodeHandle* n, GimbalLimits limits) : m_dev_path(dev_path), m_baudrate(baudrate), nh(*n), m_limits(limits),
    m_isConnected(false), m_angle_control_flag(true)
    {
        if (m_dev_path == "")
        {
            std::cout << "Please provide a non-empty device path." << std::endl;
            throw;
        }
        m_pub_imu_angle = nh.advertise<geometry_msgs::PoseStamped>("gimbal_imu_angles", 1);
        m_pub_enc_angle = nh.advertise<geometry_msgs::PoseStamped>("gimbal_enc_angles", 1);
    }

    ~GimbalController()
    {
        close(m_device);
    };

    bool connect()
    {
        if (m_isConnected == true)
        {
            close(m_device);
            m_isConnected = false;
        }

        if (setupSerial() == 0)
        {            
            m_com_obj.initialize(m_device);
            m_sbgc_parser.init(&m_com_obj);
            std::cout << "Connected." << std::endl;
            m_isConnected = true;
            return m_isConnected;
        }
        else
        {
            //std::cout << "Could not open com port." << std::endl;
            return m_isConnected;
        } 
    }

    int setupSerial()
    {   
        //std::cout << "Attempting to connect to " + m_dev_path << std::endl;            
        m_device = open(m_dev_path.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

        if (m_device < 0)
        {
            printf("Error %i from open: %s\n", errno, strerror(errno));
            return -1;
        }

        if (setupSerialParams() != 0)
        {
            printf("Error setting up com port.");
            return -1;
        }
        return 0;
    }

    int setupSerialParams()
    {
        int speed = getBaud(m_baudrate);
        int parity = 0, should_block = 0;

        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (m_device, &tty) != 0)
        {
                printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = should_block ? 1 : 0;            // read doesn't block
        tty.c_cc[VTIME] = 10;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (m_device, TCSANOW, &tty) != 0)
        {
                printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
                return -1;
        }
        fcntl(m_device, F_SETFL, FNDELAY);
        //printf("Number from fcntl: %d", num);


        return 0;
    }

    int requestData()
    {
	    SerialCommand cmd;
        cmd.init(SBGC_CMD_REALTIME_DATA_4);
        cmd.writeByte(50); //Stream rate Hz
        for ( int g = 0; g < 20 ; g++) //Pad to have cmd 21 bytes long.
        {
            cmd.writeByte(0);
        }
	    return m_sbgc_parser.send_cmd(cmd, 0);
    }

    uint8_t processData()
    {
        uint8_t error = 1;
        while(m_sbgc_parser.read_cmd())
        {
            SerialCommand &cmd = m_sbgc_parser.in_cmd;
            //std::cout << "cmd.id " << cmd.id << std::endl;

            switch(cmd.id)
            {
                case SBGC_CMD_REALTIME_DATA_3:
                    error = SBGC_cmd_realtime_data_unpack(m_rt_data, cmd);
                    //std::cout << "SBGC_CMD_REALTIME_DATA_3" << std::endl;

                break;
                case SBGC_CMD_REALTIME_DATA_4:
                    error = SBGC_cmd_realtime_data_unpack(m_rt_data, cmd);
                    //std::cout << "SBGC_CMD_REALTIME_DATA_4" << std::endl;

                break;
                default:
                    //std::cout << "default" << std::endl;

                    return 2;
            }
        }
        //std::cout << "error " << error << std::endl;
        //printf("%d\n", error);

        return error;
    }

    int setRPY(double& roll, double& pitch, double& yaw)
    {
        checkLimits(roll,pitch,yaw);
        SBGC_cmd_control_t c;
        c.mode = SBGC_CONTROL_MODE_ANGLE;
        c.speedROLL = 5*SBGC_SPEED_SCALE;
        c.speedPITCH = 45*SBGC_SPEED_SCALE;
        c.speedYAW = 60*SBGC_SPEED_SCALE;
        c.angleROLL = SBGC_DEGREE_TO_ANGLE(roll*180/3.141592654); //Changed rad to deg
        c.anglePITCH = -SBGC_DEGREE_TO_ANGLE(pitch*180/3.141592654); //Changed rad to deg, changed sign
        c.angleYAW = SBGC_DEGREE_TO_ANGLE(yaw*180/3.141592654); //Changed rad to deg
        return SBGC_cmd_control_send(c,m_sbgc_parser);
    }

    int setRPYspd(const double& roll_rate, const double& pitch_rate, const double& yaw_rate)
    {
        SBGC_cmd_control_t c;
        c.mode = SBGC_CONTROL_MODE_SPEED;
        c.speedROLL = roll_rate * SBGC_SPEED_SCALE;
        c.speedPITCH = pitch_rate * SBGC_SPEED_SCALE;
        c.speedYAW = yaw_rate * SBGC_SPEED_SCALE;        
        return SBGC_cmd_control_send(c,m_sbgc_parser);
    }

    void checkLimits(double& roll, double& pitch, double&yaw)
    {
        //Limit roll angles
        if(roll > m_limits.max_roll*M_PI/180)
        {
            roll = m_limits.max_roll*M_PI/180;
        }
        else if(roll < m_limits.min_roll*M_PI/180)
        {
            roll = m_limits.min_roll*M_PI/180;
        }
        //Limit pitch angles
        if(pitch > m_limits.max_pitch*M_PI/180)
        {
            pitch = m_limits.max_pitch*M_PI/180;
        }
        else if(pitch < m_limits.min_pitch*M_PI/180)
        {
            pitch = m_limits.min_pitch*M_PI/180;
        }
        //Limit yaw angles
        if(yaw > m_limits.max_yaw*M_PI/180)
        {
            yaw = m_limits.max_yaw*M_PI/180;
        }
        else if(yaw < m_limits.min_yaw*M_PI/180)
        {
            yaw = m_limits.min_yaw*M_PI/180;
        }
    }

    std::vector<double> getIMUAnglesRadians()
    {
        std::vector<double> imu_angles;
        imu_angles.push_back(m_rt_data.imu_angle[0]*0.02197265625*M_PI/180);//Roll
        imu_angles.push_back(m_rt_data.imu_angle[1]*0.02197265625*M_PI/180);//Pitch
        imu_angles.push_back(m_rt_data.imu_angle[2]*0.02197265625*M_PI/180);//Yaw
        return imu_angles;
    }
    
    std::vector<double> getIMUAnglesDegrees()
    {
        std::vector<double> imu_angles;
        imu_angles.push_back(m_rt_data.imu_angle[0]*0.02197265625);//Roll
        imu_angles.push_back(m_rt_data.imu_angle[1]*0.02197265625);//Pitch
        imu_angles.push_back(m_rt_data.imu_angle[2]*0.02197265625);//Yaw
        return imu_angles;
    }

    std::vector<double> getENCAnglesRadians()
    {
        std::vector<double> enc_angles;
        enc_angles.push_back(m_rt_data.rotor_angle[0]*0.02197265625*M_PI/180);//Roll
        enc_angles.push_back(m_rt_data.rotor_angle[1]*0.02197265625*M_PI/180);//Pitch
        enc_angles.push_back(m_rt_data.rotor_angle[2]*0.02197265625*M_PI/180);//Yaw
        return enc_angles;
    }

    std::vector<double> getENCAnglesDegrees()
    {
        std::vector<double> enc_angles;
        enc_angles.push_back(m_rt_data.rotor_angle[0]*0.02197265625);//Roll
        enc_angles.push_back(m_rt_data.rotor_angle[1]*0.02197265625);//Pitch
        enc_angles.push_back(m_rt_data.rotor_angle[2]*0.02197265625);//Yaw
        return enc_angles;
    }

    void printAngles()
    {
        auto imu_angles = getIMUAnglesDegrees();
        auto enc_angles = getENCAnglesDegrees();

        std::cout << "--------------------" << std::endl;
        std::cout << "IMU Angles Degrees (RPY)" << std::endl;

        for (double angle : imu_angles)
        {
            std::cout << angle << "\t";
        }
        std::cout << "\nENC Angles Degrees (RPY)\n";
        for (double angle : enc_angles)
        {
            std::cout << angle << "\t";
        }
        std::cout << std::endl;
    }

    void publishAngles()
    {
        auto imu_angles = getIMUAnglesRadians();
        auto enc_angles = getENCAnglesRadians();
        
        tf2::Quaternion q;        
        q.setRPY(imu_angles.at(0), -imu_angles.at(1), imu_angles.at(2)); //Changed sign

        geometry_msgs::PoseStamped gm_msg;
        gm_msg.header.stamp = ros::Time::now();
        gm_msg.header.frame_id = "gimbal";
        gm_msg.pose.orientation.w = q.getW();
        gm_msg.pose.orientation.x = q.getX();
        gm_msg.pose.orientation.y = q.getY();
        gm_msg.pose.orientation.z = q.getZ();

        m_pub_imu_angle.publish(gm_msg);

        q.setRPY(enc_angles.at(0), enc_angles.at(1), enc_angles.at(2));        
        gm_msg.pose.orientation.w = q.getW();
        gm_msg.pose.orientation.x = q.getX();
        gm_msg.pose.orientation.y = q.getY();
        gm_msg.pose.orientation.z = q.getZ();

        m_pub_enc_angle.publish(gm_msg);
    }

private:
    std::string m_dev_path;
    int m_device;
    int m_baudrate;
    ros::NodeHandle nh;
    ros::Publisher m_pub_imu_angle;
    ros::Publisher m_pub_enc_angle;

    bool m_isConnected;
    bool m_angle_control_flag;

    SBGC_ComObj_Imp m_com_obj;
    SBGC_Parser m_sbgc_parser;
    SBGC_cmd_realtime_data_t m_rt_data;

    GimbalLimits m_limits;
};
