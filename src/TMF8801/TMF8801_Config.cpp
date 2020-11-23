/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2020 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-TMF8801/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include "TMF8801_Config.h"

#include "TMF8801_Const.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace TMF8801
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

TMF8801_Config::TMF8801_Config(TMF8801_Io & io)
: _io{io}
{

}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void TMF8801_Config::loadCalibData(CalibData const & calib_data)
{
  _io.write(Register::FACTORY_CALIB_0, calib_data.data(), calib_data.size());
}

void TMF8801_Config::loadAlgoState(AlgoState const & algo_state)
{
  _io.write(Register::STATE_DATA_WR_0, algo_state.data(), algo_state.size());
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* TMF8801 */
