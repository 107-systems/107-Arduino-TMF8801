/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2020 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-TMF8801/graphs/contributors.
 */

#ifndef ARDUINO_TMF8801_TMF8801_API_H_
#define ARDUINO_TMF8801_TMF8801_API_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include "TMF8801_Io.h"

#include "TMF8801_Types.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace TMF8801
{

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

enum class InterruptSource
{
  ObjectDectectionAvailable, RawHistogramAvailable
};

enum class Application
{
  Unknown, Measurement, Bootloader
};

enum class RegisterContent
{
  Unknown, CalibrationData, SerialNumber, CommandResult, RawHistogram
};

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class TMF8801_Api
{

public:

  TMF8801_Api(TMF8801_Io & io, TMF8801::DelayFunc delay);


  /* Application independent API
   */
  Error           reset();
  Error           loadApplication();
  Error           loadBootloader();
  void            clearInterrupt(InterruptSource const src);
  void            enableInterrupt(InterruptSource const src);
  void            disableInterrupt(InterruptSource const src);

  Application     getCurrentApplication();
  RegisterContent getRegisterContent();
  uint8_t         getAppRevisionMajor();
  uint8_t         getAppRevisionMinor();
  uint8_t         getAppRevisionPatch();


  /* Application API
   */
  void application_readObjectDetectionResult(ObjectDetectionData & data);
  void application_loadCalibData(CalibData const & calib_data);
  void application_loadAlgoState(AlgoState const & algo_state);


  /* Bootloader API
   */
  BOOTLOADER_STATUS bootloader_download_init();
  BOOTLOADER_STATUS bootloader_set_address(uint16_t const addr);
  BOOTLOADER_STATUS bootloader_write_ram(uint8_t const * ram_firmware, size_t const ram_firmware_bytes);
  BOOTLOADER_STATUS bootloader_ramremap_reset();

private:

  TMF8801_Io & _io;
  TMF8801::DelayFunc _delay;

  BOOTLOADER_STATUS bootloader_command_transfer(BootloaderCommand & bl_cmd);
  BOOTLOADER_STATUS bootloader_wait_ready();
  BOOTLOADER_STATUS bootloader_get_status();
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* TMF8801 */

#endif /* ARDUINO_TMF8801_TMF8801_CONTROL_H_ */
