/**
 * @file i2c.hpp
 * @author duckstarr
 * @brief Class used to configure I2C protocol and enable read/write to the bus
 * @brief Reference:
 *        https://www.cmrr.umn.edu/~strupp/serial.html
 *        https://www.kernel.org/doc/Documentation/i2c/dev-interface
 * 
 */

#ifndef I2C_HPP
#define I2C_HPP

#include <iostream>

namespace sensor
{
  class I2C
  {
    public:
      /**
       * @brief Configure I2C interface
       * 
       * @param iDevicePath    Device path (i.e., /dev/i2c-1)
       * @param iDeviceAddress Device I2C address
       */
      I2C(const char* iDevicePath, uint8_t iDeviceAddress);

      /**
       * @brief Virtual destructor
       * 
       */
      virtual ~I2C();

      /**
       * @brief Disable copy constructor
       * 
       * @param iI2C 
       */
      I2C(I2C& iI2C) = delete;

      /**
       * @brief Disable copy assignment
       * 
       * @param iI2C 
       */
      void operator=(I2C& iI2C) = delete;

    protected:
      /**
       * @brief Write to I2C bus
       * 
       * @param iBuffer    Buffer containing a series of commands
       * @param iNBytes    Size of buffer (in bytes)
       * @return [ssize_t] Size of buffer written to the I2C bus
       */
      ssize_t command(const void* iBuffer, size_t iNBytes);

      /**
       * @brief Read from I2C bus
       * 
       * @param oBuffer    Response from I2C bus
       * @param iNBytes    Size of buffer (in bytes)
       * @return [ssize_t] Size of buffer read from the I2C bus
       */
      ssize_t receive(void* oBuffer, size_t iNBytes);

    private:
      /**
       * @brief Configure I2C interface
       * 
       * @return [int] File discriptor
       */
      int SetupInterface();

    private:
      int         mFd;            // File discriptor
      const char* mDevicePath;    // Device path (i.e., /dev/i2c-1)
      uint8_t     mDeviceAddress; // Device I2C address
  };
}

#endif /* I2C_HPP */
