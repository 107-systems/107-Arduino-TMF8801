/**
 * @brief Arduino library for interfacing with the TMF8801 time-of-flight distance sensor.
 * @license LGPL 3.0
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

typedef std::function<void(uint8_t const)>                           I2cStartFunc;
typedef std::function<void(uint8_t const)>                           I2cWriteFunc;
typedef std::function<void()>                                        I2cStopFunc;
typedef std::function<void(uint8_t const, uint8_t *, uint8_t const)> I2cRequestFromFunc;

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class TMF8801_Io
{
public:

  TMF8801_Io(I2cStartFunc start, I2cWriteFunc write, I2cStopFunc stop, I2cRequestFromFunc request_from, uint8_t const i2c_slave_addr);


  uint8_t read  (Register const reg);
  void    write (Register const reg, uint8_t const val);


private:

  I2cStartFunc _start;
  I2cWriteFunc _write;
  I2cStopFunc _stop;
  I2cRequestFromFunc _request_from;
  uint8_t const _i2c_slave_addr;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* TMF8801 */

#endif /* ARDUINO_TMF8801_TMF8801_IO_H_ */
