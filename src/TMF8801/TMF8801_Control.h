/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2020 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-TMF8801/graphs/contributors.
 */

#ifndef ARDUINO_TMF8801_TMF8801_CONTROL_H_
#define ARDUINO_TMF8801_TMF8801_CONTROL_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include "TMF8801_Io.h"

#include "TMF8801_Types.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace TMF8801
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class TMF8801_Control
{

public:

  TMF8801_Control(TMF8801_Io & io);


  void reset();
  void loadApplication();
  void loadBootloader();

  void readObjectDetectionResult(ObjectDetectionData & data);


private:

  TMF8801_Io & _io;

  static unsigned int const RESET_TIMEOUT_ms = 100;

};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* TMF8801 */

#endif /* ARDUINO_TMF8801_TMF8801_CONTROL_H_ */
