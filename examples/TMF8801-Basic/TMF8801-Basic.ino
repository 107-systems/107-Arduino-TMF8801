/**
 * @brief Basic example demonstrating usage of 107-Arduino-TMF8801 library.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <Wire.h>

#include <ArduinoTMF8801.h>

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static uint8_t const TMF8801_I2C_SLAVE_ADDR = (0x41 << 1);

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

void i2c_start(uint8_t const i2c_slave_addr);
void i2c_write(uint8_t const data);
void i2c_stop();
void i2c_requestFrom(uint8_t const i2c_slave_addr, uint8_t * data, uint8_t const num_bytes);

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

TMF8801::TMF8801_Io io(i2c_start, i2c_write, i2c_stop, i2c_requestFrom, TMF8801_I2C_SLAVE_ADDR);

/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/

void setup()
{
  Serial.begin(9600);
  while(!Serial) { }

  /* Setup Wire access */
  Wire.begin();

  /* Read the ID of the chip */
  uint8_t const chip_id = io.read(TMF8801::Register::ID); /* Should by 0x07 */
}

void loop()
{

}

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

void i2c_start(uint8_t const i2c_slave_addr)
{
  Wire.beginTransmission(i2c_slave_addr);
}

void i2c_write(uint8_t const data)
{
  Wire.write(data);
}

void i2c_stop()
{
  Wire.endTransmission();
}

void i2c_requestFrom(uint8_t const i2c_slave_addr, uint8_t * data, uint8_t const num_bytes)
{
  Wire.requestFrom(i2c_slave_addr, num_bytes);
  
  for(uint8_t bytes_read = 0;
      (bytes_read < num_bytes) && Wire.available();
      bytes_read++)
  {
    data[bytes_read] = Wire.read();
  }
}
