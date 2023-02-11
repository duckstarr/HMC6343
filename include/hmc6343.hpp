/**
 * @file HMC6343.hpp
 * @author duckstarr
 * @brief A C++ program used to interface with the HMC6343 module.
 * 
 */

#ifndef HMC6343_HPP
#define HMC6343_HPP

#include <i2c.hpp>
#include <register.hpp>

#include <thread>

namespace sensor
{
  class hmc6343 : public I2C
  {
    public:
      /**
       * @brief Construct a new hmc6343 object
       * 
       * @param device 
       * @param iDeviceAddress 
       */
      hmc6343(const char * device, uint8_t iDeviceAddress = HMC6343_I2C_ADDR);
      
      /**
       * @brief Virtual destructor
       * 
       */
      virtual ~hmc6343();

      /**
       * @brief Disable copy constructor
       * 
       * @param ihmc6343 
       */
      hmc6343(hmc6343& ihmc6343) = delete;

      /**
       * @brief Disable copy assignment
       * 
       * @param ihmc6343 
       */
      void operator=(hmc6343& ihmc6343) = delete;
      
      /**
       * @brief Read the raw calculated heading values from HMC6343.
       * 
       */
      void readHeading();

      /**
       * @brief Read the raw calculated magnetometer values from HMC6343.
       * 
       */
      void readMag();

      /**
       * @brief Read the raw calculated accelerometer values from HMC6343.
       * 
       */
      void readAccel();

      /**
       * @brief Read the raw calculated tilt values from HMC6343.
       * 
       */
      void readTilt();

      /**
       * @brief The register informs you of current calculation status, filter status, modes enabled and the orientation selected.
       * 
       * @return uint8_t 
       */
      uint8_t readOPMode1();

      /**
       * @brief Send enter standby mode I2C command to HMC6343.
       * 
       */
      void enterStandbyMode();

      /**
       * @brief Send enter sleep mode I2C command to HMC6343.
       * 
       */
      void enterSleepMode();

      /**
       * @brief Send enter calibration mode I2C command to HMC6343.
       * 
       */
      void enterUserCalibrationMode();

      /**
       * @brief Send exit standby (enter run) mode I2C command to HMC6343.
       * 
       */
      void exitStandbyMode();

      /**
       * @brief Send exit sleep mode I2C command to HMC6343.
       * 
       */
      void exitSleepMode();

      /**
       * @brief Send exit calibration mode I2C command to HMC6343.
       * 
       */
      void exitUserCalibrationMode();

      /**
       * @brief Start the HMC6343 calibration process.
       * 
       */
      void startCalibration();

      /**
       * @brief Configure the HMC6343 filter.
       * 
       */
      void setFilter();

    public:
      int16_t heading;     // Heading orientation
      int16_t pitch;       // Pitch orientation
      int16_t roll;        // Roll orientation
      int16_t magX;        // Magnetic field in X domain
      int16_t magY;        // Magnetic field in Y domain
      int16_t magZ;        // Magnetic field in Z domain
      int16_t accelX;      // Acceleration vector in X domain
      int16_t accelY;      // Acceleration vector in Y domain
      int16_t accelZ;      // Acceleration vector in Z domain
      int16_t temperature; // Temperature

    private:
      /**
       * @brief Write the filter value (used for weighted averages of sensor reads) to the EEPROM of the HMC6343
       *        FILTER_LSB register can be set between 0x00 and 0x0F
       *        It averages the last X sensor readings (filter register value) with the latest reading
       *        Example: if set to 4 (instead of default 0), it would average the last 4 readings with the latest reading 
       *        for sensor values such as heading with the last 4 for a total of a second average (5Hz)
       * 
       * @param reg 
       * @param data 
       */
      void writeEEPROM(uint8_t reg, uint8_t data);

      /**
       * @brief Set the physical orientation of the HMC6343 IC to either LEVEL, SIDEWAYS, or FLATFRONT
       *        This allows the IC to calculate a proper heading, pitch, and roll in tenths of degrees
       *        LEVEL      X = forward, +Z = up (default)
       *        SIDEWAYS   X = forward, +Y = up
       *        FLATFRONT  Z = forward, -X = up
       * 
       * @param orientation 
       */
      void setOrientation(uint8_t orientation);

      /**
       * @brief Read from EEPROM register
       * 
       * @param reg 
       * @return uint8_t 
       */
      uint8_t readEEPROM(uint8_t reg);

      /**
       * @brief Write to HMC6343 register
       * 
       * @param command 
       */
      void writeToRegister(uint8_t command);

      /**
       * @brief Read the returns from HMC6343 register
       * 
       * @param command 
       * @param first 
       * @param second 
       * @param third 
       */
      void readGeneric(uint8_t command, int16_t * first, int16_t * second, int16_t * third);
      
      /**
       * @brief Clear data buffer
       * 
       */
      void clearRawData();

    private:
      uint8_t rawData[6]; // Raw data buffer
      uint8_t _addr;      // Device address
  };
}

#endif /* HMC6343_HPP */
