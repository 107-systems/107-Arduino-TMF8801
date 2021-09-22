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
//#define DBG_ENABLE_VERBOSE
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

TMF8801::TMF8801_Io tmf8801_io(i2c_generic_write, i2c_generic_read, TMF8801::DEFAULT_I2C_ADDR);

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
  tmf8801_io.modify(Register::ENABLE, bm(ENABLE::CPU_RESET), bm(ENABLE::CPU_RESET));
  delay(100);
  if (!tmf8801_io.isBitSet(Register::ENABLE, bp(ENABLE::CPU_READY))) {
    DBG_ERROR("Error, CPU not ready after reset");
    return;
  }


  /* Load application stored in ROM firmware.
   */
  tmf8801_io.write(Register::APPREQID, to_integer(APPREQID::APP));
  delay(100);
  if (tmf8801_io.read(Register::APPID) != to_integer(APPID::APP)) {
    DBG_ERROR("Error, could not load application");
    return;
  }


  /* Obtain major, minor, patch number of current ROM firmware.
   */
  uint8_t const app_rev_major = tmf8801_io.read(Register::APPREV_MAJOR);
  uint8_t const app_rev_minor = tmf8801_io.read(Register::APPREV_MINOR);
  uint8_t const app_rev_patch = tmf8801_io.read(Register::APPREV_PATCH);
  DBG_INFO("ROM Firmware = %d.%d.%d", app_rev_major, app_rev_minor, app_rev_patch);


  /* Load bootloader stored in ROM firmware.
   */
  tmf8801_io.write(Register::APPREQID, to_integer(APPREQID::BOOTLOADER));
  delay(100);
  if (tmf8801_io.read(Register::APPID) != to_integer(APPID::BOOTLOADER)) {
    DBG_ERROR("Error, could not load bootloader");
    return;
  }

  auto bootloader_checksum_func = [](BootloaderCommand_SingleParameter const & bl_cmd) -> uint8_t
  {
    uint8_t cs = bl_cmd.field.bl_cmd_stat;
    cs += bl_cmd.field.bl_size;
    cs += bl_cmd.field.bl_data;
    cs ^= 0xFF;
    return cs;
  };

  auto bootloader_checksum_func_2 = [](BootloaderCommand_DualParameter const & bl_cmd) -> uint8_t
  {
    uint8_t cs = bl_cmd.field.bl_cmd_stat;
    cs += bl_cmd.field.bl_size;
    cs += bl_cmd.field.bl_data_0;
    cs += bl_cmd.field.bl_data_1;
    cs ^= 0xFF;
    return cs;
  };

  auto bootloader_command_func = [](BootloaderCommand_SingleParameter const & bl_cmd)
  {
    tmf8801_io.write(Register::BL_CMD_STAT, bl_cmd.buf, sizeof(bl_cmd.buf));
    
    DBG_INFO("CMD_STAT %02X SIZE %02X DATA[0] %02X CSUM %02X",
             bl_cmd.field.bl_cmd_stat,
             bl_cmd.field.bl_size,
             bl_cmd.field.bl_data,
             bl_cmd.field.bl_csum);
  };

  auto bootloader_command_func_2 = [](BootloaderCommand_DualParameter const & bl_cmd)
  {
    tmf8801_io.write(Register::BL_CMD_STAT, bl_cmd.buf, sizeof(bl_cmd.buf));
    
    DBG_INFO("CMD_STAT %02X SIZE %02X DATA[0] %02X DATA[1] %02X CSUM %02X",
             bl_cmd.field.bl_cmd_stat,
             bl_cmd.field.bl_size,
             bl_cmd.field.bl_data_0,
             bl_cmd.field.bl_data_1,
             bl_cmd.field.bl_csum);
  };

  auto bootloader_command_status = []() -> uint8_t
  {
    uint8_t buf[3] = {0};
    tmf8801_io.read(Register::BL_CMD_STAT, buf, sizeof(buf));
    DBG_INFO("CMD_STAT %02X SIZE %02X DATA[0] %02X", buf[0], buf[1], buf[2]);
    return buf[0];
  };

  /* Initialize TMF8801 RAM for download. */
  BootloaderCommand_SingleParameter BOOTLOADER_COMMAND_DOWNLOAD_INIT;
  BOOTLOADER_COMMAND_DOWNLOAD_INIT.field.bl_cmd_stat = to_integer(BOOTLOADER_COMMAND::DOWNLOAD_INIT);
  BOOTLOADER_COMMAND_DOWNLOAD_INIT.field.bl_size = 1;
  BOOTLOADER_COMMAND_DOWNLOAD_INIT.field.bl_data = 0x29;
  BOOTLOADER_COMMAND_DOWNLOAD_INIT.field.bl_csum = bootloader_checksum_func(BOOTLOADER_COMMAND_DOWNLOAD_INIT);

  bootloader_command_func(BOOTLOADER_COMMAND_DOWNLOAD_INIT);
  bootloader_command_status();

  /* Setup address pointer. */
  BootloaderCommand_DualParameter BOOTLOADER_COMMAND_ADDR_RAM;
  BOOTLOADER_COMMAND_ADDR_RAM.field.bl_cmd_stat = to_integer(BOOTLOADER_COMMAND::ADDR_RAM);
  BOOTLOADER_COMMAND_ADDR_RAM.field.bl_size = 2;
  BOOTLOADER_COMMAND_ADDR_RAM.field.bl_data_0 = 0x02;
  BOOTLOADER_COMMAND_ADDR_RAM.field.bl_data_1 = 0x00;
  BOOTLOADER_COMMAND_ADDR_RAM.field.bl_csum = bootloader_checksum_func_2(BOOTLOADER_COMMAND_ADDR_RAM);

  bootloader_command_func_2(BOOTLOADER_COMMAND_ADDR_RAM);
  bootloader_command_status();

  /* Write firmware. */

  /* RAMREMAP_RESET */
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
