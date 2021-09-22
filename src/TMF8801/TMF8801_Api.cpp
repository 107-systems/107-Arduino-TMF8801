/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2020 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-TMF8801/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include "TMF8801_Api.h"

#include "TMF8801_Const.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace TMF8801
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

TMF8801_Api::TMF8801_Api(TMF8801_Io & io)
: _io{io}
{

}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void TMF8801_Api::reset()
{
  _io.modify(Register::ENABLE, bm(ENABLE::CPU_RESET), bm(ENABLE::CPU_RESET));
}

void TMF8801_Api::loadApplication()
{
  _io.write(Register::APPREQID, to_integer(APPREQID::APP));
}

void TMF8801_Api::loadBootloader()
{
  _io.write(Register::APPREQID, to_integer(APPREQID::BOOTLOADER));
}

void TMF8801_Api::clearInterrupt(InterruptSource const src)
{
  if (src == InterruptSource::ObjectDectectionAvailable)
    _io.write(Register::INT_STATUS, bm(INT_STATUS::INT1));
  else
    _io.write(Register::INT_STATUS, bm(INT_STATUS::INT2));
}

void TMF8801_Api::enableInterrupt(InterruptSource const src)
{
  if (src == InterruptSource::ObjectDectectionAvailable)
    _io.modify(Register::INT_ENAB, bm(INT_ENAB::INT1), bm(INT_ENAB::INT1));
  else
    _io.modify(Register::INT_ENAB, bm(INT_ENAB::INT2), bm(INT_ENAB::INT2));
}

void TMF8801_Api::disableInterrupt(InterruptSource const src)
{
  if (src == InterruptSource::ObjectDectectionAvailable)
    _io.modify(Register::INT_ENAB, bm(INT_ENAB::INT1), 0);
  else
    _io.modify(Register::INT_ENAB, bm(INT_ENAB::INT2), 0);
}

bool TMF8801_Api::isCpuReady()
{
  return _io.isBitSet(Register::ENABLE, bp(ENABLE::CPU_READY));
}

Application TMF8801_Api::getCurrentApplication()
{
  uint8_t const appid_val = _io.read(Register::APPID);

  if      (appid_val == to_integer(APPID::APP))
    return Application::Measurement;
  else if (appid_val == to_integer(APPID::BOOTLOADER))
    return Application::Bootloader;
  else
    return Application::Unknown;
}

RegisterContent TMF8801_Api::getRegisterContent()
{
  uint8_t const register_contents_val = _io.read(Register::REGISTER_CONTENTS);

  if      (register_contents_val == to_integer(REGISTER_CONTENTS::CALIB_DATA))
    return RegisterContent::CalibrationData;
  else if (register_contents_val == to_integer(REGISTER_CONTENTS::SERIAL_NUMBER))
    return RegisterContent::SerialNumber;
  else if (register_contents_val == to_integer(REGISTER_CONTENTS::CMD_RESULT))
    return RegisterContent::CommandResult;
  else if ((register_contents_val >= 0x80) && (register_contents_val >= 0x93))
    return RegisterContent::RawHistogram;
  else
    return RegisterContent::Unknown;
}

uint8_t TMF8801_Api::getAppRevisionMajor()
{
  return _io.read(Register::APPREV_MAJOR);
}

uint8_t TMF8801_Api::getAppRevisionMinor()
{
  return _io.read(Register::APPREV_MINOR);
}

uint8_t TMF8801_Api::getAppRevisionPatch()
{
  return _io.read(Register::APPREV_PATCH);
}

void TMF8801_Api::readObjectDetectionResult(ObjectDetectionData & data)
{
  _io.read(Register::RESULT_NUMBER, data.buf, sizeof(data.buf));
}

void TMF8801_Api::loadCalibData(CalibData const & calib_data)
{
  _io.write(Register::FACTORY_CALIB_0, calib_data.data(), calib_data.size());
}

void TMF8801_Api::loadAlgoState(AlgoState const & algo_state)
{
  _io.write(Register::STATE_DATA_WR_0, algo_state.data(), algo_state.size());
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* TMF8801 */
