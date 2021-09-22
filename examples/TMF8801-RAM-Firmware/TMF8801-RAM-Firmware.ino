/**
 * @brief Performing a firmware upload to the TMF8801 RAM as
 * well as remapping and starting the RAM firmware.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <Wire.h>

#include <107-Arduino-TMF8801.h>

#define DBG_ENABLE_ERROR
#define DBG_ENABLE_WARNING
#define DBG_ENABLE_INFO
#define DBG_ENABLE_DEBUG
#define DBG_ENABLE_VERBOSE
#include <ArduinoDebug.hpp>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

using namespace TMF8801;
using namespace drone;

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

void i2c_generic_write(uint8_t const i2c_slave_addr, uint8_t const reg_addr, uint8_t const * buf, uint8_t const num_bytes);
void i2c_generic_read (uint8_t const i2c_slave_addr, uint8_t const reg_addr, uint8_t       * buf, uint8_t const num_bytes);

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

DEBUG_INSTANCE(120, Serial);

TMF8801_Io tmf8801_io(i2c_generic_write, i2c_generic_read, TMF8801::DEFAULT_I2C_ADDR);
TMF8801_Api tmf8801_api(tmf8801_io, delay);

/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/

void setup()
{
  /* Setup Serial */
  Serial.begin(9600);
  while(!Serial) { }

  DBG_INFO("TMF8801 RAM Firmware Upload");

  /* Setup Wire (I2C) */
  Wire.begin();


  /* Read chip id to determine if a connection to
   * the TMF8801 sensor can be established at the
   * given address.
   */
  if (tmf8801_io.read(Register::ID) != ID_EXPECTED_ID) {
    DBG_ERROR("Error, could not connect to TMF8801");
    return;
  }


  /* Perform a reset.
   */
  tmf8801_api.reset();
  delay(100);
  if (!tmf8801_api.isCpuReady()) {
    DBG_ERROR("Error, CPU not ready after reset");
    return;
  }


  /* Load application stored in ROM firmware.
   */
  tmf8801_api.loadApplication();
  delay(100);
  if (tmf8801_api.getCurrentApplication() != Application::Measurement) {
    DBG_ERROR("Error, could not load application");
    return;
  }


  /* Obtain major, minor, patch number of current ROM firmware.
   */
  DBG_INFO("ROM Firmware = %d.%d.%d", tmf8801_api.getAppRevisionMajor(), tmf8801_api.getAppRevisionMinor(), tmf8801_api.getAppRevisionPatch());


  /* Load bootloader stored in ROM firmware.
   */
  tmf8801_api.loadBootloader();
  delay(100);
  if (tmf8801_api.getCurrentApplication() != Application::Bootloader) {
    DBG_ERROR("Error, could not load bootloader");
    return;
  }


  /* Download RAM firmware to TMF8801.
   */
  BOOTLOADER_STATUS bl_status = BOOTLOADER_STATUS::READY;

  DBG_INFO("bootloader_download_init() ...");
  if ((bl_status = tmf8801_api.bootloader_download_init()) != BOOTLOADER_STATUS::READY)
  {
    DBG_ERROR("bootloader_download_init() failed with %d", to_integer(bl_status));
    return;
  }

  DBG_INFO("bootloader_set_address() ...");
  if ((bl_status = tmf8801_api.bootloader_set_address(0x0000)) != BOOTLOADER_STATUS::READY)
  {
    DBG_ERROR("bootloader_set_address() failed with %d", to_integer(bl_status));
    return;
  }

  DBG_INFO("bootloader_write_ram() ...");
  if ((bl_status = tmf8801_api.bootloader_write_ram(main_app_3v3_k2_bin, sizeof(main_app_3v3_k2_bin))) != BOOTLOADER_STATUS::READY)
  {
    DBG_ERROR("bootloader_write_ram() failed with %d", to_integer(bl_status));
    return;
  }

  DBG_INFO("bootloader_ramremap_reset() ...");
  if ((bl_status = tmf8801_api.bootloader_ramremap_reset()) != BOOTLOADER_STATUS::READY)
  {
    DBG_ERROR("bootloader_ramremap_reset() failed with %d", to_integer(bl_status));
    return;
  }

  delay(100);
  if (!tmf8801_api.isCpuReady()) {
    DBG_ERROR("Error, CPU not ready after reset");
    return;
  }

  /* Obtain major, minor, patch number of current RAM firmware.
   */
  DBG_INFO("RAM Firmware = %d.%d.%d", tmf8801_api.getAppRevisionMajor(), tmf8801_api.getAppRevisionMinor(), tmf8801_api.getAppRevisionPatch());
}

void loop()
{

}

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

void i2c_generic_write(uint8_t const i2c_slave_addr, uint8_t const reg_addr, uint8_t const * buf, uint8_t const num_bytes)
{
  Wire.beginTransmission(i2c_slave_addr);
  Wire.write(reg_addr);
  for(uint8_t bytes_written = 0; bytes_written < num_bytes; bytes_written++) {
    Wire.write(buf[bytes_written]);
  }
  Wire.endTransmission();
}

void i2c_generic_read(uint8_t const i2c_slave_addr, uint8_t const reg_addr, uint8_t * buf, uint8_t const num_bytes)
{
  Wire.beginTransmission(i2c_slave_addr);
  Wire.write(reg_addr);
  Wire.endTransmission();

  Wire.requestFrom(i2c_slave_addr, num_bytes);
  for(uint8_t bytes_read = 0; (bytes_read < num_bytes) && Wire.available(); bytes_read++) {
    buf[bytes_read] = Wire.read();
  }
}
