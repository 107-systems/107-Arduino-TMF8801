/**
 * @brief Performing a factory calibration of the TMF8801 ToF sensor
 * utilizing the 107-Arduino-TMF8801 library.
 *
 * The device has to be factory calibrated to be able to report the
 * correct distances in the final environment.
 *   Calibration Environment:
 *     - Device has to be in the final (correct) optical stack
 *     - Clear glass (no smudge on the glass)
 *     - No target in front of the device within 40 cm (see datasheet)
 *     - Dark room or low ambient light
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <Wire.h>

#include <107-Arduino-TMF8801.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

using namespace TMF8801;
using namespace drone;

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static uint8_t const TMF8801_I2C_ADDR = TMF8801::DEFAULT_I2C_ADDR;

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

void i2c_generic_write(uint8_t const i2c_slave_addr, uint8_t const reg_addr, uint8_t const * buf, uint8_t const num_bytes);
void i2c_generic_read (uint8_t const i2c_slave_addr, uint8_t const reg_addr, uint8_t       * buf, uint8_t const num_bytes);

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

TMF8801::TMF8801_Io tmf8801_io(i2c_generic_write, i2c_generic_read, TMF8801_I2C_ADDR);

/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/

void setup()
{
  /* Setup Serial */
  Serial.begin(9600);
  while(!Serial) { }
  Serial.println("TMF8801 Factory Calibration");

  /* Setup Wire (I2C) */
  Wire.begin();

  /* Read chip id to determine if a connection to
   * the TMF8801 sensor can be established at the
   * given address.
   */
  if (tmf8801_io.read(Register::ID) != ID_EXPECTED_ID) {
    Serial.println("Error, could not connect to TMF8801");
    return;
  }

  /* Perform a reset.
   */
  tmf8801_io.modify(Register::ENABLE, bm(ENABLE::CPU_RESET), bm(ENABLE::CPU_RESET));
  delay(100);
  if (!tmf8801_io.isBitSet(Register::ENABLE, bp(ENABLE::CPU_READY))) {
    Serial.println("Error, CPU not ready after reset");
    return;
  }

  /* Load application.
   */
  tmf8801_io.write(Register::APPREQID, to_integer(APPREQID::APP));
  delay(100);
  if (tmf8801_io.read(Register::APPID) != to_integer(APPID::APP)) {
    Serial.println("Error, could not load application");
    return;
  }

  /* Start factory calibration.
   */
  tmf8801_io.write(Register::COMMAND, to_integer(COMMAND::FACTORY_CALIB));

  /* Wait for factory calibration to be complete
   */
  for (unsigned long const calib_start = millis();
       (millis() - calib_start) < 10000;
       delay(100))
  {
    if (tmf8801_io.read(Register::REGISTER_CONTENTS) == to_integer(REGISTER_CONTENTS::CALIB_DATA))
    {
      Serial.println("Calibration complete");
      /* Reading all 14 bytes of obtained factory calibration data.
       */
      Serial.print("FACTORY_CALIB_[0-13] = {0x");
      uint8_t factory_calib_data[14] = {0};
      tmf8801_io.read(Register::FACTORY_CALIB_0, factory_calib_data, sizeof(factory_calib_data));
      for (size_t b = 0; b < sizeof(factory_calib_data); b++)
      {
        Serial.print(factory_calib_data[b], HEX);
        if (b < (sizeof(factory_calib_data) - 1))
          Serial.print(", 0x");
      }
      Serial.println("}");
      /* Exit sketch here, calibration complete.
       */
      return;
    }
  }

  /* A timeout has occurred, let's check the status register for
   * the error cause.
   */
  Serial.println("Timeout during calibration");
  Serial.print  ("STATUS = 0x");
  Serial.println(tmf8801_io.read(Register::STATUS), HEX);
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
