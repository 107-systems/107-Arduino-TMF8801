/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2020 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-TMF8801/graphs/contributors.
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

TMF8801_Io::TMF8801_Io(I2cWriteFunc write, I2cReadFunc read, uint8_t const i2c_slave_addr)
: _write{write}
, _read{read}
, _i2c_slave_addr{i2c_slave_addr}
{

}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

uint8_t TMF8801_Io::read(Register const reg)
{
  uint8_t data = 0;
  read(reg, &data, 1);
  return data;
}

void TMF8801_Io::write(Register const reg, uint8_t const val)
{
  write(reg, &val, 1);
}

void TMF8801_Io::read(Register const reg, uint8_t * buf, size_t const bytes)
{
  _read(_i2c_slave_addr, to_integer(reg), buf, bytes);
}

void TMF8801_Io::write(Register const reg, uint8_t const * buf, size_t const bytes)
{
  _write(_i2c_slave_addr, to_integer(reg), buf, bytes);
}

void TMF8801_Io::modify(Register const reg, uint8_t const bitmask, uint8_t const val)
{
  uint8_t reg_val = read(reg);
  reg_val &= ~(bitmask);
  reg_val |= (val & bitmask);
  write(reg, reg_val);
}

bool TMF8801_Io::isBitSet(Register const reg, uint8_t const bitpos)
{
  uint8_t const reg_val = read(reg);
  if (reg_val & (1<<bitpos))
    return true;
  else
    return false;
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* TMF8801 */
