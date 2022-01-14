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
, _gpio_control{0}
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

  return start_continuous_measurement(measurement_period_ms);
}

bool ArduinoTMF8801::start_continuous_measurement(uint8_t const measurement_period_ms)
{
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
  _io.write(TMF8801::Register::CMD_DATA5, _gpio_control);
  _io.write(TMF8801::Register::CMD_DATA4, 0x00); /* No GPIO control used */
  _io.write(TMF8801::Register::CMD_DATA3, 0x00); /* Needs to be always 00 */
  _io.write(TMF8801::Register::CMD_DATA2, measurement_period_ms);
  _io.write(TMF8801::Register::CMD_DATA1, 0xFF); /* Needs to be always ff */
  _io.write(TMF8801::Register::CMD_DATA0, 0xFF); /* Needs to be always ff */
  _io.write(TMF8801::Register::COMMAND,   TMF8801::to_integer(TMF8801::COMMAND::DISTANCE_MEASURE_MODE_1)); /* Set flag to perform target distance measurement with 8 bytes of data containing where including setting of calibration (and algorithm state) configuration. */

  if(measurement_period_ms > 0)
  {
    unit::Time const update_period = (static_cast<float>(measurement_period_ms) / 1000.0) * unit::second;
    unit::Frequency const update_rate = 1.0 / update_period;
    setUpdateRate(update_rate);
    return true;
  }

  _error = TMF8801::Error::Param;
  return false;
}

uint32_t ArduinoTMF8801::read_serial_number()
{
  _io.write(TMF8801::Register::COMMAND, TMF8801::to_integer(TMF8801::COMMAND::SERIAL_NUMBER_READOUT));
  delay(5);
  uint32_t const serial_number = _api.application_read_serial_number();
  return serial_number;
}

void ArduinoTMF8801::set_gpio(TMF8801::GPIO const gpio)
{
  if(gpio==TMF8801::GPIO::GPIO0) _gpio_control=((_gpio_control)&0xF0)|0x05;
  if(gpio==TMF8801::GPIO::GPIO1) _gpio_control=((_gpio_control)&0x0F)|0x50;
  /* Set gpio control setting without actually performing a measurement as commands 0x02 or 0x03 would do
   */
  _io.write(TMF8801::Register::CMD_DATA0, _gpio_control);
  _io.write(TMF8801::Register::COMMAND,   TMF8801::to_integer(TMF8801::COMMAND::SET_GPIO_CONTROL_SETTING)); /* Set flag to set GPIO control setting */
}

void ArduinoTMF8801::clr_gpio(TMF8801::GPIO const gpio)
{
  if(gpio==TMF8801::GPIO::GPIO0) _gpio_control=((_gpio_control)&0xF0)|0x04;
  if(gpio==TMF8801::GPIO::GPIO1) _gpio_control=((_gpio_control)&0x0F)|0x40;
  /* Set gpio control setting without actually performing a measurement as commands 0x02 or 0x03 would do
   */
  _io.write(TMF8801::Register::CMD_DATA0, _gpio_control);
  _io.write(TMF8801::Register::COMMAND,   TMF8801::to_integer(TMF8801::COMMAND::SET_GPIO_CONTROL_SETTING)); /* Set flag to set GPIO control setting */
}

void ArduinoTMF8801::stop_continuous_measurement()
{
  _io.write(TMF8801::Register::COMMAND,   TMF8801::to_integer(TMF8801::COMMAND::STOP_CONTINUOUS_MEASUREMENT)); /* Set flag to stop everything */
}

void ArduinoTMF8801::change_i2c_address(uint8_t const new_address)
{
  uint8_t new_address_buf = (new_address & 0x7F) << 1;    /* Shift one bit */
  new_address_buf = new_address_buf & 0xFE;               /* Set Bit 0 to 0 */
  _io.write(TMF8801::Register::CMD_DATA1, new_address_buf);
  _io.write(TMF8801::Register::CMD_DATA0, 0x00);          /* Needs to be always 0 */
  _io.write(TMF8801::Register::COMMAND,   TMF8801::to_integer(TMF8801::COMMAND::CHANGE_I2C_ADDRESS)); /* Set flag to change i2c address */

  _io.set_i2c_slace_addr(new_address);
}

bool ArduinoTMF8801::update_available()
{
  /* check for firmware 3.0.18 */
  uint8_t revMajor=_api.getAppRevisionMajor();
  uint8_t revMinor=_api.getAppRevisionMinor();
  uint8_t revPatch=_api.getAppRevisionPatch();

  if(revMajor<3) return true;
  else if(revMajor==3)
  {
    if(revMinor==0)
    {
      if(revPatch<18) return true;
      else return false;
    }
    else return false;
  }
  else return false;
}

bool ArduinoTMF8801::perform_update()
{
  /* Load bootloader stored in ROM firmware.
   */
  if ((_error = _api.loadBootloader()) != TMF8801::Error::None)
  {
//    DBG_ERROR("loadBootloader() failed with %d", to_integer(error));
    return false;
  }


  /* Download RAM firmware to TMF8801.
   */
  TMF8801::BOOTLOADER_STATUS bl_status = TMF8801::BOOTLOADER_STATUS::READY;

//  DBG_INFO("bootloader_download_init() ...");
  if ((bl_status = _api.bootloader_download_init()) != TMF8801::BOOTLOADER_STATUS::READY)
  {
//    DBG_ERROR("bootloader_download_init() failed with %d", to_integer(bl_status));
    return false;
  }

//  DBG_INFO("bootloader_set_address() ...");
  if ((bl_status = _api.bootloader_set_address(0x0000)) != TMF8801::BOOTLOADER_STATUS::READY)
  {
//    DBG_ERROR("bootloader_set_address() failed with %d", to_integer(bl_status));
    return false;
  }

//  DBG_INFO("bootloader_write_ram() ...");
  if ((bl_status = _api.bootloader_write_ram(TMF8801::main_app_3v3_k2_bin, sizeof(TMF8801::main_app_3v3_k2_bin))) != TMF8801::BOOTLOADER_STATUS::READY)
  {
//    DBG_ERROR("bootloader_write_ram() failed with %d", to_integer(bl_status));
    return false;
  }

//  DBG_INFO("bootloader_ramremap_reset() ...");
  if ((bl_status = _api.bootloader_ramremap_reset()) != TMF8801::BOOTLOADER_STATUS::READY)
  {
//    DBG_ERROR("bootloader_ramremap_reset() failed with %d", to_integer(bl_status));
    return false;
  }

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
