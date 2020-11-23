/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2020 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-TMF8801/graphs/contributors.
 */

#ifndef ARDUINO_TMF8801_TMF8801_CONFIG_H_
#define ARDUINO_TMF8801_TMF8801_CONFIG_H_

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

class TMF8801_Config
{

public:

  TMF8801_Config(TMF8801_Io & io);


  void loadCalibData(CalibData const & calib_data);
  void loadAlgoState(AlgoState const & algo_state);


private:

  TMF8801_Io & _io;

};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* TMF8801 */

#endif /* ARDUINO_TMF8801_TMF8801_CONFIG_H_ */
