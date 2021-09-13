/**
 * @brief Basic example demonstrating usage of 107-Arduino-TMF8801 library.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <Wire.h>

#include <107-Arduino-TMF8801.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

using namespace drone;

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static int const TMF8801_INT_PIN = 6;

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

void i2c_generic_write(uint8_t const i2c_slave_addr, uint8_t const reg_addr, uint8_t const * buf, uint8_t const num_bytes);
void i2c_generic_read (uint8_t const i2c_slave_addr, uint8_t const reg_addr, uint8_t       * buf, uint8_t const num_bytes);

/**************************************************************************************
 * GLOBAL CONSTANTS
 **************************************************************************************/

/* The calibration data needs to be obtained for every sensor by
 * executing the sketch TMF8801-FactoryCalib with the sensor mounted
 * in the target environment.
 */
#warning "Run 'TMF8801-FactoryCalib' once in order to obtain sensor calibration data for constant TMF8801_CALIB_DATA"
static TMF8801::CalibData const TMF8801_CALIB_DATA{0x31, 0x9E, 0x0, 0xB6, 0x9, 0xE0, 0xFB, 0xF7, 0xF8, 0xF1, 0xE3, 0xC7, 0x7, 0xFC};
static TMF8801::AlgoState const TMF8801_ALGO_STATE{0xB1, 0xA9, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t const MEASUREMENT_PERIOD_ms = 100;

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

ArduinoTMF8801 tmf8801(i2c_generic_write,
                       i2c_generic_read,
                       delay,
                       TMF8801::DEFAULT_I2C_ADDR,
                       TMF8801_CALIB_DATA,
                       TMF8801_ALGO_STATE,
                       [](unit::Length const distance)
                       {
                         char msg[32];
                         snprintf(msg, sizeof(msg), "Distance = %0.3f m", distance.value());
                         Serial.println(msg);
                       });

/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/

void setup()
{
  Serial.begin(9600);
  while(!Serial) { }

  /* Print data of TMF8801 sensor. */
  Serial.print(tmf8801);

  /* Setup Wire access */
  Wire.begin();

  /* Attach interrupt handler */
  pinMode(TMF8801_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(TMF8801_INT_PIN), [](){ tmf8801.onExternalEventHandler(); }, FALLING);

  /* Configure TMF8801 */
  if (!tmf8801.begin(MEASUREMENT_PERIOD_ms))
  {
    Serial.print("ArduinoTMF8801::begin(...) failed, error code ");
    Serial.print((int)tmf8801.error());
    for(;;) { }
  }

  Serial.println("TMF8801 OK");
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
