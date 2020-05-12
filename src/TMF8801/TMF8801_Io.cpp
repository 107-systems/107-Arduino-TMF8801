/**
 * @brief Arduino library for interfacing with the TMF8801 time-of-flight distance sensor.
 * @license LGPL 3.0
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include "TMF8801_Io.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace TMF8801
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

TMF8801_Io::TMF8801_Io(I2cStartFunc start, I2cWriteFunc write, I2cStopFunc stop, I2cRequestFromFunc request_from, uint8_t const i2c_slave_addr)
: _start{start}
, _write{write}
, _stop{stop}
, _request_from{request_from}
, _i2c_slave_addr{i2c_slave_addr}
{

}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

uint8_t TMF8801_Io::read(Register const reg)
{
  _start(_i2c_slave_addr);
  _write(to_integer(reg));
  uint8_t data = 0;
  _request_from(_i2c_slave_addr, &data, 1);
  return data;
}

void TMF8801_Io::write(Register const reg, uint8_t const val)
{
  _start(_i2c_slave_addr);
  _write(to_integer(reg));
  _write(val);
  _stop();
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* TMF8801 */
