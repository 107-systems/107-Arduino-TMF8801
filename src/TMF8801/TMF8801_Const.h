/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2020 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-TMF8801/graphs/contributors.
 */

#ifndef ARDUINO_TMF8801_TMF8801_CONST_H_
#define ARDUINO_TMF8801_TMF8801_CONST_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <stdint.h>

#include <type_traits>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace TMF8801
{

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

enum class Register : uint8_t
{
  APPID                  = 0x00,
  APPREQID               = 0x02,

  APPREV_MAJOR           = 0x01,
  APPREV_MINOR           = 0x12,
  APPREV_PATCH           = 0x13,

  CMD_DATA9              = 0x06,
  CMD_DATA8              = 0x07,
  CMD_DATA7              = 0x08,
  CMD_DATA6              = 0x09,
  CMD_DATA5              = 0x0A,
  CMD_DATA4              = 0x0B,
  CMD_DATA3              = 0x0C,
  CMD_DATA2              = 0x0D,
  CMD_DATA1              = 0x0E,
  CMD_DATA0              = 0x0F,
  COMMAND                = 0x10,
  PREVIOUS               = 0x11,
  STATUS                 = 0x1D,
  REGISTER_CONTENTS      = 0x1E,
  TID                    = 0x1F,

  RESULT_NUMBER          = 0x20,
  RESULT_INFO            = 0x21,
  DISTANCE_PEAK_0        = 0x22,
  DISTANCE_PEAK_1        = 0x23,
  SYS_CLOCK_0            = 0x24,
  SYS_CLOCK_1            = 0x25,
  SYS_CLOCK_2            = 0x26,
  SYS_CLOCK_3            = 0x27,
  STATE_DATA_0           = 0x28,
  STATE_DATA_1           = 0x29,
  STATE_DATA_2           = 0x2A,
  STATE_DATA_3           = 0x2B,
  STATE_DATA_4           = 0x2C,
  STATE_DATA_5           = 0x2D,
  STATE_DATA_6           = 0x2E,
  STATE_DATA_7           = 0x2F,
  STATE_DATA_8_XTALK_MSB = 0x30,
  STATE_DATA_9_XTALK_LSB = 0x31,
  STATE_DATA_10_TJ       = 0x32,
  REFERENCE_HITS_0       = 0x33,
  REFERENCE_HITS_1       = 0x34,
  REFERENCE_HITS_2       = 0x35,
  REFERENCE_HITS_3       = 0x36,
  OBJECT_HITS_0          = 0x37,
  OBJECT_HITS_1          = 0x38,
  OBJECT_HITS_2          = 0x39,
  OBJECT_HITS_3          = 0x3A,

  FACTORY_CALIB_0        = 0x20,
  STATE_DATA_WR_0        = 0x2E,

  ENABLE                 = 0xE0,
  INT_STATUS             = 0xE1,
  INT_ENAB               = 0xE2,
  ID                     = 0xE3,
  REVID                  = 0xE4,

  BL_CMD_STAT            = 0x08,
  BL_SIZE                = 0x09,
  BL_DATA                = 0x0A,
  BL_CSUM                = 0x0B,
};

enum class APPID : uint8_t
{
  APP        = 0xC0,
  BOOTLOADER = 0x80,
};

enum class APPREQID : uint8_t
{
  APP        = 0xC0,
  BOOTLOADER = 0x80,
};

enum class ENABLE : uint8_t
{
  CPU_RESET = 7,
  CPU_READY = 6,
};

enum class COMMAND : uint8_t
{
  DISTANCE_MEASURE_MODE_1  = 0x02, /* Set flag to perform target distance measurement with 8 bytes of data containing where including setting of calibration (and algorithm state) configuration. */
  FACTORY_CALIB            = 0x0A, /* Perform factory calibration in the final customer application including cover glass, no ambient light and no target. */
  DOWNLOAD_CALIB_AND_STATE = 0x0B,
};

enum class STATUS : uint8_t
{
  Idle                 = 0x00, /* No error, information that internal state machine is idling. */
  Diagnostic           = 0x01, /* No error, information that internal state machine is in diagnostic mode. */
  Start                = 0x02, /* No error, internal state machine is in initialization phase. */
  Calibration          = 0x03, /* No error, internal state machine is in the calibration phase. */
  LightCol             = 0x04, /* No error, internal state machine is performing HW measurements and running the proximity algorithm. */
  Algorithm            = 0x05, /* No error, internal state machine is running the distance algorithm. */
  Startup              = 0x06, /* No error, internal state machine is initializing HW and SW. */

  VcselPwrFail         = 0x10, /* Error, eye safety check failed, VCSEL is disabled by HW circuit. */
  VcselLedAFail        = 0x11, /* Error, eye safety check failed for anode. VCSEL is disabled by HW circuit. */
  VcselLedKFail        = 0x12, /* Error, eye safety check failed for cathode. VCSEL is disabled by HW circuit. */
  InvalidParam         = 0x18, /* Error, internal program error. A parameter to a function call was out of range. */
  InvalidDevice        = 0x19, /* A status information that a measurement got interrupted. Is +only used internally. Not populated to I2C register. */
  CalibError           = 0x1B, /* Error, electrical calibration failed. No two peaks found to calibrate. */
  InvalidCommand       = 0x1C, /* Error, either a command byte was written to register 0x10 that is not supported by the application, or the command was sent while the application was busy executing the previous command. I.e. no stop command was sent before. */
  InvalidState         = 0x1D, /* Error, internal program error. The constant state table is faulty. */
  ErrAlgorithm         = 0x1F, /* Internal error in algorithm. */
  InvalidData          = 0x23, /* Only used in test modes. */
  HalInterrupted       = 0x24, /* Same as 0x19 */
  ErrMissingFactCal    = 0x27, /* There is no (or no valid) factory calibration on the device. Using default values instead. */
  ErrInvalidFactCal    = 0x28, /* Parsing of the provided factory calibration found an illegal calibration value. */
  ErrInvalidAlgState   = 0x29, /* Parsing of the provided algorithm state found an illegal algorithm state value. */
  ErrInvalidProxConfig = 0x2A, /* Only used in test modes. */
  ErrInvalidDistConfig = 0x2B, /* Only used in test modes */
};

enum class REGISTER_CONTENTS : uint8_t
{
  CALIB_DATA    = 0x0A,
  SERIAL_NUMBER = 0x47,
  CMD_RESULT    = 0x55,
};

enum class INT_STATUS : uint8_t
{
  INT1 = 0,
  INT2 = 1,
};

enum class INT_ENAB : uint8_t
{
  INT1 = 0,
  INT2 = 1,
};

enum class BOOTLOADER_COMMAND : uint8_t
{
  RAMREMAP_RESET = 0x11, /* Remap RAM to Address 0 and Reset */
  DOWNLOAD_INIT  = 0x14, /* Initialize for RAM download from host to TMF8801 */
  W_RAM          = 0x41, /* Write RAM Region (Plain = not encoded into e.g. Intel Hex Records) */
  ADDR_RAM       = 0x43, /* Set the read/write RAM pointer to a given address */
};

enum class BOOTLOADER_STATUS : uint8_t
{
  READY               = 0x00, /* Bootloader is ready to receive a new command */
  ERR_SIZE            = 0x01, /* The size field has an invalid number (e.g. Reset command must have size set to 0, any other value will lead to this error) and bootloader is ready to receive a new command */
  ERR_CSUM            = 0x02, /* The Checksum is wrong and the bootloader is ready to receive a new command */
  ERR_RES             = 0x03, /* The command given is not supported by the bootloader and the bootloader is ready to receive a new command. */
  ERR_APP             = 0x04, /* Application switch not supported and the bootloader is ready to receive a new command. */
  ERR_TIMEOUT         = 0x05, /* Timeout occurred and the bootloader is ready to receive a new command. */
  ERR_LOCK            = 0x06, /* The command cannot be executed on an encrypted device and the bootloader is ready to receive a new command. */
  ERR_RANGE           = 0x07, /* The specified address is out of range or the command would lead to a read/write to an out-of-bounds address and the bootloader is ready to receive a new command. */
  ERR_MORE            = 0x08, /* The command was executed but did not lead to a success. For each command that can return this error code, the section specifies how to interpret the additional information. The bootloader is ready to receive a new command. */
  ERROR_GENERAL_START = 0x09, /* Unspecified error and the bootloader is ready to receive a new command. */
  ERROR_GENERAL_STOP  = 0x0F,
  BUSY_START          = 0x10, /* Bootloader cannot receive a new command – wait for status to become READY or ERROR */
  BUSY_STOP           = 0xFF,
};

enum class Error : int
{
  None    =  0,
  Timeout = -1,
  ChipId  = -2,
};

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static uint8_t constexpr ID_EXPECTED_ID = 0x07;
static uint8_t constexpr DEFAULT_I2C_ADDR = 0x41;

/**************************************************************************************
 * CONVERSION FUNCTIONS
 **************************************************************************************/

template <typename Enumeration>
constexpr auto to_integer(Enumeration const value) -> typename std::underlying_type<Enumeration>::type
{
  return static_cast<typename std::underlying_type<Enumeration>::type>(value);
}

template <typename Enumeration>
constexpr auto bp(Enumeration const value) -> typename std::underlying_type<Enumeration>::type
{
  return to_integer(value);
}

template <typename Enumeration>
constexpr auto bm(Enumeration const value) -> typename std::underlying_type<Enumeration>::type
{
  return (1 << to_integer(value));
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* TMF8801 */

#endif /* ARDUINO_TMF8801_TMF8801_CONST_H_ */
