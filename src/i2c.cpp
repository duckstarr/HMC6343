/**
 * @file i2c_communication.cpp
 * @author duckstarr
 * @brief Class used to configure I2C protocol and enable read/write to the bus
 * @date 2022-10-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <i2c.hpp>

#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <termios.h> /* POSIX terminal control definitions */

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

namespace sensor
{
  I2C::I2C(const char* iDevicePath, uint8_t iDeviceAddress)
    : mDevicePath(iDevicePath)
    , mDeviceAddress(iDeviceAddress)
  {
    mFd = SetupInterface();
  }

  I2C::~I2C()
  {
    close(mFd);
  }

  int I2C::SetupInterface()
  {
    int wFd;

    if((wFd = open(mDevicePath, O_RDWR)) < 0)
    {
      printf("HMC6343::SetupInterface - Error failed to open I2C bus [%s].\n", mDevicePath);
      exit(-1);
    }
    else
    {
      fcntl(wFd, F_SETFL, 0);
    }

    if(ioctl(wFd, I2C_SLAVE, mDeviceAddress) < 0)
    {
      printf("HMC6343::SetupInterface - Error can't find sensor at I2C address [0x%02X].\n", mDeviceAddress);
      exit(-1);
    }

    struct termios wTty;

    tcgetattr(wFd, &wTty);
    cfsetispeed(&wTty, B115200);
    cfsetospeed(&wTty, B115200);

    // tty.c_cflag &= ~PARENB;
    // tty.c_cflag &= ~CSTOPB;
    wTty.c_cflag &= ~CSIZE;
    wTty.c_cflag |= CS8;

    tcsetattr (wFd, TCSANOW, &wTty);

    return wFd;
  }

  ssize_t I2C::command(const void* iBuffer, size_t iNBytes)
  {
    return write(mFd, iBuffer, iNBytes);
  }

  ssize_t I2C::receive(void* iBuffer, size_t iNBytes)
  {
    return read(mFd, iBuffer, iNBytes);
  }
}
