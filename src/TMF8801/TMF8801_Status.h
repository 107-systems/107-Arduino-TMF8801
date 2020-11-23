/**
 * @brief Arduino library for interfacing with the TMF8801 time-of-flight distance sensor.
 * @license LGPL 3.0
 */

#ifndef ARDUINO_TMF8801_TMF8801_STATUS_H_
#define ARDUINO_TMF8801_TMF8801_STATUS_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include "TMF8801_Io.h"

#include "TMF8801_Const.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace TMF8801
{

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

enum class Application
{
  Unkown, Measurement, Bootloader
};

enum class RegisterContent
{
  Unkown, CalibrationData, SerialNumber, CommandResult, RawHistogram
};

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class TMF8801_Status
{

public:

  TMF8801_Status(TMF8801_Io & io);

  bool            isCpuReady();
  Application     currentApplication();
  RegisterContent getRegisterContent();


private:

  TMF8801_Io & _io;

};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* TMF8801 */

#endif /* ARDUINO_TMF8801_TMF8801_STATUS_H_ */
