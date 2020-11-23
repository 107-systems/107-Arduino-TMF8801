/**
 * @brief Arduino library for interfacing with the TMF8801 time-of-flight distance sensor.
 * @license LGPL 3.0
 */

#ifndef ARDUINO_TMF8801_TMF8801_TYPES_H_
#define ARDUINO_TMF8801_TMF8801_TYPES_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <stdint.h>

#undef max
#undef min
#include <array>
#include <functional>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace TMF8801
{

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

typedef std::function<void(unsigned long)> DelayFunc;

static size_t constexpr CALIB_DATA_SIZE = 14;
typedef std::array<uint8_t, CALIB_DATA_SIZE> CalibData;

static size_t constexpr ALGO_STATE_SIZE = 11;
typedef std::array<uint8_t, ALGO_STATE_SIZE> AlgoState;

union ObjectDetectionData
{
  struct __attribute__((packed))
  {
    uint8_t  result_number;
    uint8_t  result_info;
    uint16_t distance_peak_0_mm;
  } field;
  uint8_t buf[sizeof(field)];
  static_assert(sizeof(buf) == 4, "Error, ObjectDetectionData size does not match expectation");
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* TMF8801 */

#endif /* ARDUINO_TMF8801_TMF8801_TYPES_H_ */
