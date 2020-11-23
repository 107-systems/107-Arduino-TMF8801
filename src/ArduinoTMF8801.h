/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2020 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-TMF8801/graphs/contributors.
 */

#ifndef ARDUINO_TMF8801_H_
#define ARDUINO_TMF8801_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#undef max
#undef min
#include <functional>

#include "TMF8801/TMF8801_Io.h"
#include "TMF8801/TMF8801_Const.h"
#include "TMF8801/TMF8801_Types.h"
#include "TMF8801/TMF8801_Config.h"
#include "TMF8801/TMF8801_Status.h"
#include "TMF8801/TMF8801_Control.h"

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static uint8_t constexpr TMF8801_DEFAULT_I2C_ADDR = 0x41;

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class ArduinoTMF8801
{

public:

  ArduinoTMF8801(TMF8801::I2cWriteFunc write,
                 TMF8801::I2cReadFunc read,
                 TMF8801::DelayFunc delay,
                 uint8_t const i2c_slave_addr,
                 TMF8801::CalibData const & calib_data,
                 TMF8801::AlgoState const & algo_state);


  bool begin();


  void           clearerr();
  TMF8801::Error error();


  bool isDataReady();
  unsigned int getDistance_mm();


private:

  TMF8801::Error _error;
  TMF8801::TMF8801_Io _io;
  TMF8801::DelayFunc _delay;
  TMF8801::TMF8801_Config _config;
  TMF8801::TMF8801_Control _ctrl;
  TMF8801::TMF8801_Status _status;
  TMF8801::CalibData const & _calib_data;
  TMF8801::AlgoState const & _algo_state;

  bool waitForCpuReady();
  bool waitForApplication();
};

#endif /* ARDUINO_TMF8801_H_ */
