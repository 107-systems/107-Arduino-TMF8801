/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2020 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-TMF8801/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include "TMF8801_Control.h"

#include "TMF8801_Const.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace TMF8801
{

/**************************************************************************************
 * CONSTANT
 **************************************************************************************/

static unsigned int constexpr TIMEOUT_INCREMENT_ms = 10;

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

TMF8801_Control::TMF8801_Control(TMF8801_Io & io)
: _io{io}
{

}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void TMF8801_Control::reset()
{
  _io.modify(Register::ENABLE, bm(ENABLE::CPU_RESET), bm(ENABLE::CPU_RESET));
}

void TMF8801_Control::loadApplication()
{
  _io.write(Register::APPREQID, to_integer(APPREQID::APP));
}

void TMF8801_Control::loadBootloader()
{
  _io.write(Register::APPREQID, to_integer(APPREQID::BOOTLOADER));
}

void TMF8801_Control::clearInterrupt(InterruptSource const src)
{
  if (src == InterruptSource::ObjectDectectionAvailable)
    _io.write(Register::INT_STATUS, bm(INT_STATUS::INT1));
  else
    _io.write(Register::INT_STATUS, bm(INT_STATUS::INT2));
}

void TMF8801_Control::enableInterrupt(InterruptSource const src)
{
  if (src == InterruptSource::ObjectDectectionAvailable)
    _io.modify(Register::INT_ENAB, bm(INT_ENAB::INT1), bm(INT_ENAB::INT1));
  else
    _io.modify(Register::INT_ENAB, bm(INT_ENAB::INT2), bm(INT_ENAB::INT2));
}

void TMF8801_Control::disableInterrupt(InterruptSource const src)
{
  if (src == InterruptSource::ObjectDectectionAvailable)
    _io.modify(Register::INT_ENAB, bm(INT_ENAB::INT1), 0);
  else
    _io.modify(Register::INT_ENAB, bm(INT_ENAB::INT2), 0);
}

void TMF8801_Control::readObjectDetectionResult(ObjectDetectionData & data)
{
  _io.read(Register::RESULT_NUMBER, data.buf, sizeof(data.buf));
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* TMF8801 */
