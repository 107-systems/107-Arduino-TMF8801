/**
 * @brief Arduino library for interfacing with the TMF8801 time-of-flight distance sensor.
 * @license LGPL 3.0
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include "TMF8801_Status.h"
#include <Arduino.h>
/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace TMF8801
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

TMF8801_Status::TMF8801_Status(TMF8801_Io & io)
: _io{io}
{

}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

bool TMF8801_Status::isCpuReady()
{
  return _io.isBitSet(Register::ENABLE, bp(ENABLE::CPU_READY));
}

Application TMF8801_Status::currentApplication()
{
  uint8_t const appid_val = _io.read(Register::APPID);

  if      (appid_val == to_integer(APPID::APP))
    return Application::Measurement;
  else if (appid_val == to_integer(APPID::BOOTLOADER))
    return Application::Bootloader;
  else
    return Application::Unkown;
}

RegisterContent TMF8801_Status::getRegisterContent()
{
  uint8_t const register_contents_val = _io.read(Register::REGISTER_CONTENTS);

  if      (register_contents_val == to_integer(REGISTER_CONTENTS::CALIB_DATA))
    return RegisterContent::CalibrationData;
  else if (register_contents_val == to_integer(REGISTER_CONTENTS::SERIAL_NUMBER))
    return RegisterContent::SerialNumber;
  else if (register_contents_val == to_integer(REGISTER_CONTENTS::CMD_RESULT))
    return RegisterContent::CommandResult;
  else if ((register_contents_val >= 0x80) && (register_contents_val >= 0x93))
    return RegisterContent::RawHistogram;
  else
    return RegisterContent::Unkown;
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* TMF8801 */
