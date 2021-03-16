
#include "main.h"
#include "i2c.h"
#include "gpio.h"
#include "sht3x_Ar.h"
extern I2C_HandleTypeDef hi2c1;
  float humidity;

  float temp;


uint16_t readStatus(void) {
  writeCommand(SHT31_READSTATUS);

  uint8_t data[3];
	HAL_I2C_Master_Receive(&hi2c1,SHT31_DEFAULT_ADDR,data,3,1000);

  uint16_t stat = data[0];
  stat <<= 8;
  stat |= data[1];
  return stat;
}

/**
 * Performs a reset of the sensor to put it into a known state.
 */
void reset(void) {
  writeCommand(SHT31_SOFTRESET);
  HAL_Delay(10);
}

/**
 * Enables or disabled the heating element.
 *
 * @param h True to enable the heater, False to disable it.
 */
void heater(bool h) {
  if (h)
    writeCommand(SHT31_HEATEREN);
  else
    writeCommand(SHT31_HEATERDIS);
  HAL_Delay(1);
}

/*!
 *  @brief  Return sensor heater state
 *  @return heater state (TRUE = enabled, FALSE = disabled)
 */
bool isHeaterEnabled() {
  uint16_t regValue = readStatus();
	return (((regValue) >> (SHT31_REG_HEATER_BIT)) & 0x01);
 // return (bool)bitRead(regValue, SHT31_REG_HEATER_BIT);
}

/**
 * Gets a single temperature reading from the sensor.
 *
 * @return A float value indicating the temperature.
 */
float  readTemperature(void) {
  if (!readTempHum())
    return NULL;

  return temp;
}

/**
 * Gets a single relative humidity reading from the sensor.
 *
 * @return A float value representing relative humidity.
 */
float readHumidity(void) {
  if (!readTempHum())
    return NULL;

  return humidity;
}

/**
 * Performs a CRC8 calculation on the supplied values.
 *
 * @param data  Pointer to the data to use when calculating the CRC8.
 * @param len   The number of bytes in 'data'.
 *
 * @return The computed CRC8 value.
 */
static uint8_t crc8(const uint8_t *data, int Size) {
  /*
   *
   * CRC-8 formula from page 14 of SHT spec pdf
   *
   * Test data 0xBE, 0xEF should yield 0x92
   *
   * Initialization data 0xFF
   * Polynomial 0x31 (x8 + x5 +x4 +1)
   * Final XOR 0x00
   */

  const uint8_t POLYNOMIAL= 0x31;
  uint8_t crc =0xFF;

  for (int j = Size; j; --j) {
    crc ^= *data++;

    for (int i = 8; i; --i) {
      crc = (crc & 0x80) ? (crc << 1) ^ POLYNOMIAL : (crc << 1);
    }
  }
  return crc;
}


/**
 * Internal function to perform a temp + humidity read.
 *
 * @return True if successful, otherwise false.
 */
bool readTempHum(void) {
  uint8_t readbuffer[6];

  writeCommand(SHT31_MEAS_HIGHREP);

  HAL_Delay(20);

  //i2c_dev->read(readbuffer, sizeof(readbuffer));
	HAL_I2C_Master_Receive(&hi2c1,SHT31_DEFAULT_ADDR,readbuffer,sizeof(readbuffer),1000);
  if (readbuffer[2] != crc8(readbuffer, 2) ||
      readbuffer[5] != crc8(readbuffer + 3, 2))
    return false;

  int32_t stemp = (int32_t)(((uint32_t)readbuffer[0] << 8) | readbuffer[1]);
  // simplified (65536 instead of 65535) integer version of:
  // temp = (stemp * 175.0f) / 65535.0f - 45.0f;
  stemp = ((4375 * stemp) >> 14) - 4500;
  temp = (float)stemp / 100.0f;

  uint32_t shum = ((uint32_t)readbuffer[3] << 8) | readbuffer[4];
  // simplified (65536 instead of 65535) integer version of:
  // humidity = (shum * 100.0f) / 65535.0f;
  shum = (625 * shum) >> 12;
  humidity = (float)shum / 100.0f;

  return true;
}

/**
 * Internal function to perform and I2C write.
 *
 * @param cmd   The 16-bit command ID to send.
 */
HAL_StatusTypeDef writeCommand(uint16_t command) {
  uint8_t cmd[2];

  cmd[0] = command >> 8;
  cmd[1] = command & 0xFF;
	return HAL_I2C_Master_Transmit(&hi2c1,SHT31_DEFAULT_ADDR,cmd,2,1000);
}
