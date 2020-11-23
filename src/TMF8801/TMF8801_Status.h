/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2020 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-TMF8801/graphs/contributors.
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
