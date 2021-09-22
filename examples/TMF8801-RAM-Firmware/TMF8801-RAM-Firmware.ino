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
TMF8801_Api tmf8801_api(tmf8801_io);

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
    DBG_VERBOSE("(%d) C %02X | S %02X | D %02X | C %02X",
                transfer_size,
                bl_cmd.field.bl_cmd_stat,
                bl_cmd.field.bl_size,
                bl_cmd.field.bl_data[0],
                bl_cmd.field.bl_data[bl_cmd.field.bl_size]);
  };

  auto bootloader_command_status = []() -> uint8_t
  {
    uint8_t buf[3] = {0};
    tmf8801_io.read(Register::BL_CMD_STAT, buf, sizeof(buf));
    DBG_INFO("CMD_STAT %02X SIZE %02X DATA[0] %02X", buf[0], buf[1], buf[2]);
    return buf[0];
  };

  /* Initialize TMF8801 RAM for download. */
  DBG_INFO("DOWNLOAD_INIT");
  {
    BootloaderCommand download_init;
    download_init.field.bl_cmd_stat = to_integer(BOOTLOADER_COMMAND::DOWNLOAD_INIT);
    download_init.field.bl_size = 1;
    download_init.field.bl_data[0] = 0x29;

    bootloader_command_transfer(download_init);
    bootloader_command_status();
  }

  /* Setup address pointer. */
  DBG_INFO("ADDR_RAM");
  {
    BootloaderCommand addr_ram;
    addr_ram.field.bl_cmd_stat = to_integer(BOOTLOADER_COMMAND::ADDR_RAM);
    addr_ram.field.bl_size = 2;
    addr_ram.field.bl_data[0] = 0x00;
    addr_ram.field.bl_data[1] = 0x00;

    bootloader_command_transfer(addr_ram);
    bootloader_command_status();
  }

  /* Write firmware. */
  DBG_INFO("W_RAM, sizeof(main_app_3v3_k2_bin) = %d bytes", sizeof(main_app_3v3_k2_bin));
  size_t const RAM_FIRWARE_BYTES_WRITTEN_PER_TRANSFER = 16;
  size_t bytes_written = 0;
  for (; bytes_written < sizeof(main_app_3v3_k2_bin); bytes_written += RAM_FIRWARE_BYTES_WRITTEN_PER_TRANSFER)
  {
    BootloaderCommand w_ram;
    w_ram.field.bl_cmd_stat = to_integer(BOOTLOADER_COMMAND::W_RAM);
    w_ram.field.bl_size = 16;
    std::copy(main_app_3v3_k2_bin + bytes_written,
              main_app_3v3_k2_bin + bytes_written + RAM_FIRWARE_BYTES_WRITTEN_PER_TRANSFER ,
              w_ram.field.bl_data);
    bootloader_command_transfer(w_ram);
    uint8_t const status = bootloader_command_status();
    DBG_VERBOSE("%d bytes written, status = %d", bytes_written, status);
  }
  /* Remove last increment before the exit of the loop. */
  bytes_written -= RAM_FIRWARE_BYTES_WRITTEN_PER_TRANSFER;
  /* Write the remaining bytes. */
  size_t const bytes_remaining = sizeof(main_app_3v3_k2_bin) - bytes_written;
  DBG_VERBOSE("%d bytes remaining, %d bytes written", bytes_remaining, bytes_written);
  if (bytes_remaining > 0)
  {
    BootloaderCommand w_ram;
    w_ram.field.bl_cmd_stat = to_integer(BOOTLOADER_COMMAND::W_RAM);
    w_ram.field.bl_size = bytes_remaining;
    std::copy(main_app_3v3_k2_bin + bytes_written,
              main_app_3v3_k2_bin + bytes_written + bytes_remaining,
              w_ram.field.bl_data);
    bootloader_command_transfer(w_ram);
    uint8_t const status = bootloader_command_status();
    bytes_written += bytes_remaining;
    DBG_VERBOSE("%d bytes written, status = %d", bytes_written, status);
  }

  /* RAMREMAP_RESET */
  DBG_INFO("RAMREMAP_RESET");
  {
    BootloaderCommand ramremap_reset;
    ramremap_reset.field.bl_cmd_stat = to_integer(BOOTLOADER_COMMAND::RAMREMAP_RESET);
    ramremap_reset.field.bl_size = 0;

    bootloader_command_transfer(ramremap_reset);
    bootloader_command_status();
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
