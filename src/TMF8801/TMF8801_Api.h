/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2020 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-TMF8801/graphs/contributors.
 */

#ifndef ARDUINO_TMF8801_TMF8801_API_H_
#define ARDUINO_TMF8801_TMF8801_API_H_

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
 * TYPEDEF
 **************************************************************************************/

enum class InterruptSource
{
  ObjectDectectionAvailable, RawHistogramAvailable
};

enum class Application
{
  Unknown, Measurement, Bootloader
};

enum class RegisterContent
{
  Unknown, CalibrationData, SerialNumber, CommandResult, RawHistogram
};

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class TMF8801_Api
{

public:

  TMF8801_Api(TMF8801_Io & io);


  /* Application independent API
   */
  void            reset();
  void            loadApplication();
  void            loadBootloader();
  void            clearInterrupt(InterruptSource const src);
  void            enableInterrupt(InterruptSource const src);
  void            disableInterrupt(InterruptSource const src);

  bool            isCpuReady();
  Application     getCurrentApplication();
  RegisterContent getRegisterContent();
  uint8_t         getAppRevisionMajor();
  uint8_t         getAppRevisionMinor();
  uint8_t         getAppRevisionPatch();


  /* Application API
   */
  void readObjectDetectionResult(ObjectDetectionData & data);
  void loadCalibData(CalibData const & calib_data);
  void loadAlgoState(AlgoState const & algo_state);


private:

  TMF8801_Io & _io;

  static unsigned int const RESET_TIMEOUT_ms = 100;

};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* TMF8801 */

#endif /* ARDUINO_TMF8801_TMF8801_CONTROL_H_ */
