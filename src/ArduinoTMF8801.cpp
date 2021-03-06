/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2020 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-TMF8801/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include "ArduinoTMF8801.h"
#include <Arduino.h>
/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

ArduinoTMF8801::ArduinoTMF8801(TMF8801::I2cWriteFunc write,
                               TMF8801::I2cReadFunc read,
                               TMF8801::DelayFunc delay,
                               uint8_t const i2c_slave_addr,
                               TMF8801::CalibData const & calib_data,
                               TMF8801::AlgoState const & algo_state)
: _error{TMF8801::Error::None}
, _io{write, read, i2c_slave_addr}
, _delay{delay}
, _config{_io}
, _ctrl{_io}
, _status{_io}
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
  _ctrl.reset();
  if (!waitForCpuReady())
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
  _ctrl.loadApplication();
  if (!waitForApplication())
    return false;

  _io.write(TMF8801::Register::COMMAND, TMF8801::to_integer(TMF8801::COMMAND::DOWNLOAD_CALIB_AND_STATE));
  _config.loadCalibData(_calib_data);
  _config.loadAlgoState(_algo_state);

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

  return true;
}

void ArduinoTMF8801::clearerr()
{
  _error = TMF8801::Error::None;
}

TMF8801::Error ArduinoTMF8801::error()
{
  return _error;
}

bool ArduinoTMF8801::isDataReady()
{
  return (_status.getRegisterContent() == TMF8801::RegisterContent::CommandResult);
}

unsigned int ArduinoTMF8801::getDistance_mm()
{
  TMF8801::ObjectDetectionData data;
  _ctrl.readObjectDetectionResult(data);
  return data.field.distance_peak_0_mm;
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

bool ArduinoTMF8801::waitForCpuReady()
{
  static unsigned int constexpr CPU_READY_TIMEOUT_ms = 100;
  static unsigned int constexpr CPU_READY_TIMEOUT_INCREMENT_ms = 10;

  /* Poll ENABLE::CPU_READY to determine if sensor is available again (ENABLE::CPU_READY = '1'). */
  unsigned int t = 0;
  for (; t < CPU_READY_TIMEOUT_ms; t += CPU_READY_TIMEOUT_INCREMENT_ms)
  {
    _delay(CPU_READY_TIMEOUT_INCREMENT_ms);
    if (_status.isCpuReady())
      return true;
  }

  /* A timeout has occurred. */
  _error = TMF8801::Error::Timeout;
  return false;
}

bool ArduinoTMF8801::waitForApplication()
{
  static unsigned int constexpr APP_LOADED_TIMEOUT_ms = 100;
  static unsigned int constexpr APP_LOADED_TIMEOUT_INCREMENT_ms = 10;

  /* Poll ENABLE::CPU_READY to determine if sensor is available again (ENABLE::CPU_READY = '1'). */
  unsigned int t = 0;
  for (; t < APP_LOADED_TIMEOUT_ms; t += APP_LOADED_TIMEOUT_INCREMENT_ms)
  {
    _delay(APP_LOADED_TIMEOUT_INCREMENT_ms);
    if (_status.currentApplication() == TMF8801::Application::Measurement)
      return true;
  }

  /* A timeout has occurred. */
  _error = TMF8801::Error::Timeout;
  return false;
}
