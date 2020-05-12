/**
 * @brief Arduino library for interfacing with the TMF8801 time-of-flight distance sensor.
 * @license LGPL 3.0
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

  ENABLE                 = 0xE0,
  INT_STATUS             = 0xE1,
  INT_ENAB               = 0xE2,
  ID                     = 0xE3,
  REVID                  = 0xE4,
};

/**************************************************************************************
 * CONVERSION FUNCTIONS
 **************************************************************************************/

template <typename Enumeration>
constexpr auto to_integer(Enumeration const value) -> typename std::underlying_type<Enumeration>::type
{
  return static_cast<typename std::underlying_type<Enumeration>::type>(value);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* TMF8801 */

#endif /* ARDUINO_TMF8801_TMF8801_CONST_H_ */
