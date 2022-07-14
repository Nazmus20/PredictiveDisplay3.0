#include "SBGC_lib/SBGC.h"
#include <iostream>
#include <sys/ioctl.h>

// Class to interface with AlexMos gimbal driver
class SBGC_ComObj_Imp : public SBGC_ComObj
{



  public:
    SBGC_ComObj_Imp()
    {
        m_initialized = false;
    }

    virtual uint16_t getBytesAvailable()
    {
        if (!m_initialized)
        {
            //std::cout << "COM Object not initialized!" << std::endl;
            return 1;
        }
            //std::cout << "COM Object initialized!" << std::endl;

        int bytes_available;
        int result = ioctl(m_device, FIONREAD, &bytes_available);
        // std::cout << "result: " << result << std::endl;
        // std::cout << "bytes_available: " << bytes_available << std::endl;

        usleep(5);
        return bytes_available;
        // return 1;
    }

    virtual uint8_t readByte()
    {
        if (!m_initialized)
        {
            //std::cout << "COM Object not initialized!" << std::endl;
            return 1;
        }
        char red;
        int err = read(m_device, &red, sizeof red);
        if (err < 0)
        {
            return err;
        }
        else
        {
            return (uint8_t)red;
        }
    }

    virtual void writeByte(uint8_t b)
    {
        if (!m_initialized)
        {
            //std::cout << "COM Object not initialized!" << std::endl;
            return;
        }
        char wry;
        memcpy(&wry, &b, sizeof b);
        write(m_device, &wry, sizeof wry);
    }
    // Arduino com port is not buffered, so empty space is unknown.
    virtual uint16_t getOutEmptySpace()
    {
        if (!m_initialized)
        {
            //std::cout << "COM Object not initialized!" << std::endl;
            return 1;
        }
        return 0xFFFF;
    }

    void initialize(int device_id)
    {
        m_device = device_id;
        m_initialized = true;
    }

    int m_device;
    bool m_initialized;
};
