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

#include <algorithm>

#include "TMF8801_Const.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace TMF8801
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

TMF8801_Api::TMF8801_Api(TMF8801_Io & io, TMF8801::DelayFunc delay)
: _io{io}
, _delay{delay}
{

}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

Error TMF8801_Api::reset()
{
  _io.modify(Register::ENABLE, bm(ENABLE::CPU_RESET), bm(ENABLE::CPU_RESET));

  static size_t constexpr CPU_READY_TIMEOUT_ms = 100;
  static size_t constexpr CPU_READY_TIMEOUT_INCREMENT_ms = 10;

  for (unsigned int t = 0; t < CPU_READY_TIMEOUT_ms; t += CPU_READY_TIMEOUT_INCREMENT_ms)
  {
    _delay(CPU_READY_TIMEOUT_INCREMENT_ms);
    if (_io.isBitSet(Register::ENABLE, bp(ENABLE::CPU_READY)))
      return Error::None;
  }

  return Error::Timeout;
}

Error TMF8801_Api::loadApplication()
{
  _io.write(Register::APPREQID, to_integer(APPREQID::APP));

  static size_t constexpr APP_LOADED_TIMEOUT_ms = 100;
  static size_t constexpr APP_LOADED_TIMEOUT_INCREMENT_ms = 10;

  for (size_t t = 0; t < APP_LOADED_TIMEOUT_ms; t += APP_LOADED_TIMEOUT_INCREMENT_ms)
  {
    _delay(APP_LOADED_TIMEOUT_INCREMENT_ms);
    if (getCurrentApplication() == TMF8801::Application::Measurement)
      return Error::None;
  }

  return Error::Timeout;
}

Error TMF8801_Api::loadBootloader()
{
  _io.write(Register::APPREQID, to_integer(APPREQID::BOOTLOADER));

  static size_t constexpr BOOTLOADER_LOADED_TIMEOUT_ms = 100;
  static size_t constexpr BOOTLOADER_LOADED_TIMEOUT_INCREMENT_ms = 10;

  for (size_t t = 0; t < BOOTLOADER_LOADED_TIMEOUT_ms; t += BOOTLOADER_LOADED_TIMEOUT_INCREMENT_ms)
  {
    _delay(BOOTLOADER_LOADED_TIMEOUT_INCREMENT_ms);
    if (getCurrentApplication() == TMF8801::Application::Bootloader)
      return Error::None;
  }

  return Error::Timeout;
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

void TMF8801_Api::application_readObjectDetectionResult(ObjectDetectionData & data)
{
  _io.read(Register::RESULT_NUMBER, data.buf, sizeof(data.buf));
}

void TMF8801_Api::application_loadCalibData(CalibData const & calib_data)
{
  _io.write(Register::FACTORY_CALIB_0, calib_data.data(), calib_data.size());
}

void TMF8801_Api::application_loadAlgoState(AlgoState const & algo_state)
{
  _io.write(Register::STATE_DATA_WR_0, algo_state.data(), algo_state.size());
}

BOOTLOADER_STATUS TMF8801_Api::bootloader_download_init()
{
  BootloaderCommand download_init;
  download_init.field.bl_cmd_stat = to_integer(BOOTLOADER_COMMAND::DOWNLOAD_INIT);
  download_init.field.bl_size = 1;
  download_init.field.bl_data[0] = 0x29;

  return bootloader_command_transfer(download_init);
}

BOOTLOADER_STATUS TMF8801_Api::bootloader_set_address(uint16_t const addr)
{
  BootloaderCommand addr_ram;
  addr_ram.field.bl_cmd_stat = to_integer(BOOTLOADER_COMMAND::ADDR_RAM);
  addr_ram.field.bl_size = 2;
  addr_ram.field.bl_data[0] = static_cast<uint8_t>((addr >> 0) & 0x00FF);
  addr_ram.field.bl_data[1] = static_cast<uint8_t>((addr >> 8) & 0x00FF);

  return bootloader_command_transfer(addr_ram);
}

BOOTLOADER_STATUS TMF8801_Api::bootloader_write_ram(uint8_t const * ram_firmware, size_t const ram_firmware_bytes)
{
  static size_t constexpr RAM_FIRWARE_BYTES_WRITTEN_PER_TRANSFER = 16;

  BOOTLOADER_STATUS bl_status = BOOTLOADER_STATUS::READY;

  BootloaderCommand w_ram;
  w_ram.field.bl_cmd_stat = to_integer(BOOTLOADER_COMMAND::W_RAM);

  size_t bytes_written = 0;
  for (; bytes_written < ram_firmware_bytes; bytes_written += RAM_FIRWARE_BYTES_WRITTEN_PER_TRANSFER)
  {
    w_ram.field.bl_size = RAM_FIRWARE_BYTES_WRITTEN_PER_TRANSFER;
    std::copy(ram_firmware + bytes_written,
              ram_firmware + bytes_written + RAM_FIRWARE_BYTES_WRITTEN_PER_TRANSFER ,
              w_ram.field.bl_data);

    if ((bl_status = bootloader_command_transfer(w_ram)) != BOOTLOADER_STATUS::READY)
      return bl_status;
  }

  /* Remove last increment before the exit of the loop. */
  bytes_written -= RAM_FIRWARE_BYTES_WRITTEN_PER_TRANSFER;

  /* Write the remaining bytes. */
  size_t const bytes_remaining = ram_firmware_bytes - bytes_written;
  if (bytes_remaining > 0)
  {
    w_ram.field.bl_size = bytes_remaining;
    std::copy(ram_firmware + bytes_written,
              ram_firmware + bytes_written + bytes_remaining,
              w_ram.field.bl_data);

    if ((bl_status = bootloader_command_transfer(w_ram)) != BOOTLOADER_STATUS::READY)
      return bl_status;
  }

  return bl_status;
}

BOOTLOADER_STATUS TMF8801_Api::bootloader_ramremap_reset()
{
  BootloaderCommand ramremap_reset;
  ramremap_reset.field.bl_cmd_stat = to_integer(BOOTLOADER_COMMAND::RAMREMAP_RESET);
  ramremap_reset.field.bl_size = 0;

  return bootloader_command_transfer(ramremap_reset);
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

BOOTLOADER_STATUS TMF8801_Api::bootloader_command_transfer(BootloaderCommand & bl_cmd)
{
  /* Ensure that the bl_size parameter is not
   * larger than the array for storing it.
   */
  bl_cmd.field.bl_size = std::min(bl_cmd.field.bl_size, static_cast<uint8_t>(BOOTLOADER_COMMAND_MAX_DATA_SIZE));

  /* Calculate checksum by iterating over all
   * elements of the bootloader command.
   */
  uint8_t cs = bl_cmd.field.bl_cmd_stat;
  cs += bl_cmd.field.bl_size;
  std::for_each(bl_cmd.field.bl_data, bl_cmd.field.bl_data + bl_cmd.field.bl_size, [&cs](uint8_t const d) { cs += d; });
  cs = ~cs;

  /* Store the checksum at the right address in the
   * command structure.
   */
  bl_cmd.field.bl_data[bl_cmd.field.bl_size] = cs;

  /* Transfer the bootloader command via I2C.
   */
  size_t const transfer_size = 1 + 1 + bl_cmd.field.bl_size + 1; /* COMMAND + SIZE + DATA + CSUM */
  _io.write(Register::BL_CMD_STAT, bl_cmd.buf, transfer_size);

  /* Wait for bootloader to either be again in READY
   * state or for presenting an error message.
   */
  return bootloader_wait_ready();
}

BOOTLOADER_STATUS TMF8801_Api::bootloader_wait_ready()
{
  static size_t constexpr BOOTLOADER_COMMAND_TIMEOUT_ms = 250;
  static size_t constexpr BOOTLOADER_COMMAND_TIMEOUT_INCREMENT_ms = 10;

  /* Poll ENABLE::CPU_READY to determine if sensor is available again (ENABLE::CPU_READY = '1'). */
  for (size_t t = 0; t < BOOTLOADER_COMMAND_TIMEOUT_ms; t += BOOTLOADER_COMMAND_TIMEOUT_INCREMENT_ms)
  {
    _delay(BOOTLOADER_COMMAND_TIMEOUT_INCREMENT_ms);
    if (bootloader_get_status() == BOOTLOADER_STATUS::READY)
      return BOOTLOADER_STATUS::READY;
  }

  return bootloader_get_status();
}

BOOTLOADER_STATUS TMF8801_Api::bootloader_get_status()
{
  uint8_t buf[3] = {0};
  _io.read(Register::BL_CMD_STAT, buf, sizeof(buf));
  return static_cast<BOOTLOADER_STATUS>(buf[0]);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* TMF8801 */
