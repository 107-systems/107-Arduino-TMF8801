/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2020 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-TMF8801/graphs/contributors.
 */

#ifndef ARDUINO_TMF8801_TMF8801_IO_H_
#define ARDUINO_TMF8801_TMF8801_IO_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#undef max
#undef min
#include <functional>

#include "TMF8801_Const.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace TMF8801
{

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

typedef std::function<void(uint8_t const, uint8_t const, uint8_t const *, uint8_t const)> I2cWriteFunc;
typedef std::function<void(uint8_t const, uint8_t const, uint8_t       *, uint8_t const)> I2cReadFunc;

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class TMF8801_Io
{
public:

  TMF8801_Io(I2cWriteFunc write, I2cReadFunc read, uint8_t const i2c_slave_addr);


  uint8_t read    (Register const reg);
  void    write   (Register const reg, uint8_t const val);
  void    read    (Register const reg, uint8_t * buf, size_t const bytes);
  void    write   (Register const reg, uint8_t const * buf, size_t const bytes);
  void    modify  (Register const reg, uint8_t const bitmask, uint8_t const val);
  bool    isBitSet(Register const reg, uint8_t const bitpos);


private:

  I2cWriteFunc _write;
  I2cReadFunc _read;

  uint8_t const _i2c_slave_addr;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* TMF8801 */

#endif /* ARDUINO_TMF8801_TMF8801_IO_H_ */
