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

  auto bootloader_command_transfer = [](BootloaderCommand & bl_cmd)
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
    tmf8801_io.write(Register::BL_CMD_STAT, bl_cmd.buf, transfer_size);
  };

  auto bootloader_command_status = []() -> uint8_t
  {
    uint8_t buf[3] = {0};
    tmf8801_io.read(Register::BL_CMD_STAT, buf, sizeof(buf));
    DBG_INFO("CMD_STAT %02X SIZE %02X DATA[0] %02X", buf[0], buf[1], buf[2]);
    return buf[0];
  };

  /* Initialize TMF8801 RAM for download. */
  BootloaderCommand download_init;
  download_init.field.bl_cmd_stat = to_integer(BOOTLOADER_COMMAND::DOWNLOAD_INIT);
  download_init.field.bl_size = 1;
  download_init.field.bl_data[0] = 0x29;

  bootloader_command_transfer(download_init);
  bootloader_command_status();

  /* Setup address pointer. */
  BootloaderCommand addr_ram;
  addr_ram.field.bl_cmd_stat = to_integer(BOOTLOADER_COMMAND::ADDR_RAM);
  addr_ram.field.bl_size = 2;
  addr_ram.field.bl_data[0] = 0x02;
  addr_ram.field.bl_data[1] = 0x00;

  bootloader_command_transfer(addr_ram);
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
