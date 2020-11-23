/**
 * @brief Arduino library for interfacing with the TMF8801 time-of-flight distance sensor.
 * @license LGPL 3.0
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
