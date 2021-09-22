/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2020 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-TMF8801/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include "107-Arduino-TMF8801.h"

#include <Arduino.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace drone
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

ArduinoTMF8801::ArduinoTMF8801(TMF8801::I2cWriteFunc write,
                               TMF8801::I2cReadFunc read,
                               TMF8801::DelayFunc delay,
                               uint8_t const i2c_slave_addr,
                               TMF8801::CalibData const & calib_data,
                               TMF8801::AlgoState const & algo_state,
                               OnLengthDataUpdateFunc func)
: LengthSensorBase("TMF8801",
                   2.5000 * unit::meter,
                   0.0020 * unit::meter,
                   0.0    * unit::hertz,
                   func)
, _error{TMF8801::Error::None}
, _io{write, read, i2c_slave_addr}
, _api{_io, delay}
, _calib_data{calib_data}
, _algo_state{algo_state}
{

}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

bool ArduinoTMF8801::begin(uint8_t const measurement_period_ms)
{
  /* Reset the board and wait for the board to come up again
   * within a predefined time period.
   */
  if ((_error = _api.reset()) != TMF8801::Error::None)
    return false;

  /* Check the CHIP ID if it matches the expected value.
   */
  if (_io.read(TMF8801::Register::ID) != TMF8801::ID_EXPECTED_ID) {
    _error = TMF8801::Error::ChipId;
    return false;
  }

  /* Load the measurement application and verify if the
   * measurement application has been successfully loaded.
   */
  if ((_error = _api.loadApplication()) != TMF8801::Error::None)
    return false;

  _io.write(TMF8801::Register::COMMAND, TMF8801::to_integer(TMF8801::COMMAND::DOWNLOAD_CALIB_AND_STATE));
  _api.application_loadCalibData(_calib_data);
  _api.application_loadAlgoState(_algo_state);

  /* Clear the interrupt to remove any remaining pending artefacts
   * then enable the interrupt for a new distance measurement available.
   */
  _api.clearInterrupt (TMF8801::InterruptSource::ObjectDectectionAvailable);
  _api.enableInterrupt(TMF8801::InterruptSource::ObjectDectectionAvailable);

  /* Configure TMF8801 according to TMF8X0X Host Driver Communication:
   * Use above configuration and configure for continuous mode, period
   * of 100 ms, GPIOs are not used, run combined proximity and distance
   * algorithm.
   */
  _io.write(TMF8801::Register::CMD_DATA7, 0x03); /* Algorithm state and factory calibration is provided */
  _io.write(TMF8801::Register::CMD_DATA6, 0x23); /* Run proximity and distance algorithm and combine histograms for distance */
  _io.write(TMF8801::Register::CMD_DATA5, 0x00); /* No GPIO control used */
  _io.write(TMF8801::Register::CMD_DATA4, 0x00); /* No GPIO control used */
  _io.write(TMF8801::Register::CMD_DATA3, 0x00); /* Needs to be always 00 */
  _io.write(TMF8801::Register::CMD_DATA2, measurement_period_ms);
  _io.write(TMF8801::Register::CMD_DATA1, 0xFF); /* Needs to be always ff */
  _io.write(TMF8801::Register::CMD_DATA0, 0xFF); /* Needs to be always ff */
  _io.write(TMF8801::Register::COMMAND,   TMF8801::to_integer(TMF8801::COMMAND::DISTANCE_MEASURE_MODE_1)); /* Set flag to perform target distance measurement with 8 bytes of data containing where including setting of calibration (and algorithm state) configuration. */

  Serial.print("STATUS = 0x");
  Serial.println(_io.read(TMF8801::Register::STATUS), HEX);

  unit::Time const update_period = (static_cast<float>(measurement_period_ms) / 1000.0) * unit::second;
  unit::Frequency const update_rate = 1.0 / update_period;
  setUpdateRate(update_rate);

  return true;
}

void ArduinoTMF8801::onExternalEventHandler()
{
  /* Clear the interrupt flag. */
  _api.clearInterrupt(TMF8801::InterruptSource::ObjectDectectionAvailable);

  /* Obtain distance data. */
  TMF8801::ObjectDetectionData data;
  _api.application_readObjectDetectionResult(data);
  _distance = (data.field.distance_peak_0_mm / 1000.0) * unit::meter;

  /* Invoke new ensor data update callback. */
  onSensorValueUpdate(_distance);
}

void ArduinoTMF8801::get(unit::Length & distance)
{
  distance = _distance;
}

void ArduinoTMF8801::clearerr()
{
  _error = TMF8801::Error::None;
}

TMF8801::Error ArduinoTMF8801::error()
{
  return _error;
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* drone */
