/**
 * @file HMC6343.cpp
 * @author duckstarr
 * @brief A C++ program used to interface with the HMC6343 module.
 * 
 */

#include <hmc6343.hpp>
#include <iostream>

namespace sensor
{
  hmc6343::hmc6343(const char* iDevicePath, uint8_t iDeviceAddress) 
    : I2C(iDevicePath, iDeviceAddress)
    , heading(0)
    , pitch(0)
    , roll(0)
    , magX(0)
    , magY(0)
    , magZ(0)
    , accelX(0)
    , accelY(0)
    , accelZ(0)
    , temperature(0)
  {
    this->exitStandbyMode();
    this->setOrientation(LEVEL);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    this->clearRawData();

    uint8_t data = 0x00;
    
    // Check for device by reading I2C address from EEPROM
    data = readEEPROM(SLAVE_ADDR);
    if (!(data == 0x32)) throw std::invalid_argument("hmc6343::hmc6343() - Failed to read EEPROM.");
  }

  hmc6343::~hmc6343()
  {
    std::cout << "End of hmc6343 program." << std::endl;
  }

  uint8_t hmc6343::readEEPROM(uint8_t reg)
  {
    uint8_t buf[2];
    buf[0] = READ_EEPROM;
    buf[1] = reg;

    if (receive(buf, (uint8_t)2) != (uint8_t)2) throw std::invalid_argument("hmc6343::readEEPROM - I2C EEPROM transaction failed.");

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    if (receive(buf, (uint8_t)1) != (uint8_t)1) throw std::invalid_argument("hmc6343::readEEPROM - I2C EEPROM read failed.");

    return buf[0];
  }

  uint8_t hmc6343::readOPMode1()
  {
    uint8_t opmode1[1];

    this->writeToRegister(POST_OPMODE1);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    receive(opmode1, (uint8_t)1);

    return opmode1[0];
  }

  void hmc6343::readGeneric(uint8_t data, int16_t * first, int16_t * second, int16_t * third)
  {
    rawData[0] = _addr;
    rawData[1] = data;

    if (command(rawData, (uint8_t)2) != (uint8_t)2) throw std::invalid_argument("hmc6343::readGeneric - I2C EEPROM transaction failed.");

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    this->clearRawData();
    
    if (receive(rawData, (uint8_t)6) != (uint8_t)6) throw std::invalid_argument("hmc6343::readGeneric - I2C EEPROM read failed.");

    // Convert 6 bytes received into 3 integers
    *first = rawData[0] << 8; // MSB -> (256 * rawData[0] + rawData[1]) / 10.0
    *first |= rawData[1];     // LSB
    *second = rawData[2] << 8;
    *second |= rawData[3];
    *third = rawData[4] << 8;
    *third |= rawData[5];
  }

  void hmc6343::setOrientation(uint8_t orientation)
  {
    if (orientation == LEVEL) writeToRegister(ORIENT_LEVEL);
    else if (orientation == SIDEWAYS) writeToRegister(ORIENT_SIDEWAYS);
    else if (orientation == FLATFRONT) writeToRegister(ORIENT_FLATFRONT);

    std::this_thread::sleep_for(std::chrono::microseconds(300));
  }

  void hmc6343::setFilter()
  {
    this->writeEEPROM(FILTER_LSB, 4); // default -> 0
    this->writeEEPROM(OP_MODE1, 0x31); // default: Disabled -> 0x11

    // To confirm, the output from EEPROM is 0x31.
    std::cout << "EEPROM Output: " << std::hex << + this->readOPMode1() << std::endl;
  }

  void hmc6343::startCalibration()
  {
    this->enterUserCalibrationMode();
    std::this_thread::sleep_for(std::chrono::microseconds(300));
    std::cout << "Enter Calibration EEPROM Output: " << std::hex << + this->readOPMode1() << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(30));

    this->exitUserCalibrationMode();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    std::cout << "Exit Calibration EEPROM Output: " << std::hex << + this->readOPMode1() << std::endl; //31

    this->setFilter();
  }

  void hmc6343::writeEEPROM(uint8_t reg, uint8_t data)
  {
    uint8_t buf[3];
    buf[0] = WRITE_EEPROM;
    buf[1] = reg;
    buf[2] = data;

    command(buf, (uint8_t)3);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  void hmc6343::writeToRegister(uint8_t data)
  {
    uint8_t buf[1];
    buf[0] = data;

    if (command(buf, (uint8_t)1) != (uint8_t)1) throw std::invalid_argument("hmc6343::writeToRegister - Failed to write a command to the register.");
  }

  void hmc6343::clearRawData()
  {
    for (uint8_t i = 0; i < 6; i++)
    {
      rawData[i] = 0;
    }
  }

  void hmc6343::readHeading()
  {
    this->readGeneric(POST_HEADING, & heading, & pitch, & roll);
  }

  void hmc6343::readTilt()
  {
      this->readGeneric(POST_TILT, & pitch, & roll, & temperature);
  }

  void hmc6343::readAccel()
  {
    this->readGeneric(POST_ACCEL, & accelX, & accelY, & accelZ);
  }

  void hmc6343::readMag()
  {
    this->readGeneric(POST_MAG, & magX, & magY, & magZ);
  }

  void hmc6343::enterSleepMode()
  {
    this->writeToRegister(ENTER_SLEEP);
  }

  void hmc6343::enterStandbyMode()
  {
    this->writeToRegister(ENTER_STANDBY);
  }

  void hmc6343::enterUserCalibrationMode()
  {
    this->writeToRegister(ENTER_CAL);
  }

  void hmc6343::exitSleepMode()
  {
    this->writeToRegister(EXIT_SLEEP);
  }

  void hmc6343::exitStandbyMode()
  {
    this->writeToRegister(ENTER_RUN);
  }

  void hmc6343::exitUserCalibrationMode()
  {
    this->writeToRegister(EXIT_CAL);
  }
}
