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

#include <107-Arduino-Sensor.hpp>

#undef max
#undef min
#include <functional>

#include "TMF8801/TMF8801_Io.h"
#include "TMF8801/TMF8801_Api.h"
#include "TMF8801/TMF8801_Const.h"
#include "TMF8801/TMF8801_Types.h"

#include "TMF8801/firmware/main_app_3v3_k2.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace drone
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class ArduinoTMF8801 : public LengthSensorBase
{

public:

  typedef std::function<void(unit::Length const)> OnLengthDataUpdateFunc;


  ArduinoTMF8801(TMF8801::I2cWriteFunc write,
                 TMF8801::I2cReadFunc read,
                 TMF8801::DelayFunc delay,
                 uint8_t const i2c_slave_addr,
                 TMF8801::CalibData const & calib_data,
                 TMF8801::AlgoState const & algo_state,
                 OnLengthDataUpdateFunc func);


  bool begin(uint8_t const measurement_period_ms);


  void onExternalEventHandler();


  virtual void get(unit::Length & distance) override;


  void           clearerr();
  TMF8801::Error error();


private:

  TMF8801::Error _error;
  TMF8801::TMF8801_Io _io;
  TMF8801::TMF8801_Api _api;
  TMF8801::CalibData const & _calib_data;
  TMF8801::AlgoState const & _algo_state;
  unit::Length _distance;

};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* drone */

#endif /* ARDUINO_TMF8801_H_ */
