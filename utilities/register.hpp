/**
 * @file register.hpp
 * @author duckstarr
 * @brief Data Sheet: https://cdn.sparkfun.com/datasheets/Sensors/Magneto/HMC6343.pdf
 * 
 */

#ifndef REGISTER_H
#define REGISTER_H

// HMC6343 I2C Address (0x32 >> 1 = 0x19)
#define HMC6343_I2C_ADDR 0x19

// HMC6343 Registers
#define SLAVE_ADDR 0x00
#define SW_VERSION 0x02
#define OP_MODE1 0x04
#define OP_MODE2 0x05
#define SN_LSB 0x06
#define SN_MSB 0x07
#define DATE_CODE_YY 0x08 
#define DATE_CODE_WW 0x09 
#define DEVIATION_LSB 0x0A
#define DEVIATION_MSB 0x0B
#define VARIATION_LSB 0x0C
#define VARIATION_MSB 0x0D
#define XOFFSET_LSB 0x0E
#define XOFFSET_MSB 0x0F
#define YOFFSET_LSB 0x10
#define YOFFSET_MSB 0x11
#define ZOFFSET_LSB 0x12
#define ZOFFSET_MSB 0x13
#define FILTER_LSB 0x14
#define FILTER_MSB 0x15

// HMC6343 Commands
#define POST_ACCEL 0x40
#define POST_MAG 0x45
#define POST_HEADING 0x50
#define POST_TILT 0x55
#define POST_OPMODE1 0x65
#define ENTER_CAL 0x71
#define ORIENT_LEVEL 0x72
#define ORIENT_SIDEWAYS 0x73
#define ORIENT_FLATFRONT 0x74
#define ENTER_RUN 0x75
#define ENTER_STANDBY 0x76
#define EXIT_CAL 0x7E
#define RESET 0x82
#define ENTER_SLEEP 0x83
#define EXIT_SLEEP 0x84
#define READ_EEPROM 0xE1
#define WRITE_EEPROM 0xF1

// HMC6343 Orientations
#define LEVEL 0     // X = forward, +Z = up (default)
#define SIDEWAYS 1  // X = forward, +Y = up
#define FLATFRONT 2 // Z = forward, -X = up

#endif /* REGISTER_H */