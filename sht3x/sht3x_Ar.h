#ifndef ADAFRUIT_SHT31_H
#define ADAFRUIT_SHT31_H

#include "main.h"
#include "i2c.h"
#include "gpio.h"
#include <stdbool.h>

#define SHT31_DEFAULT_ADDR (0x44<<1) /**< SHT31 Default Address */
#define SHT31_MEAS_HIGHREP_STRETCH                                             \
  0x2C06 /**< Measurement High Repeatability with Clock Stretch Enabled */
#define SHT31_MEAS_MEDREP_STRETCH                                              \
  0x2C0D /**< Measurement Medium Repeatability with Clock Stretch Enabled */
#define SHT31_MEAS_LOWREP_STRETCH                                              \
  0x2C10 /**< Measurement Low Repeatability with Clock Stretch Enabled*/
#define SHT31_MEAS_HIGHREP                                                     \
  0x2400 /**< Measurement High Repeatability with Clock Stretch Disabled */
#define SHT31_MEAS_MEDREP                                                      \
  0x240B /**< Measurement Medium Repeatability with Clock Stretch Disabled */
#define SHT31_MEAS_LOWREP                                                      \
  0x2416 /**< Measurement Low Repeatability with Clock Stretch Disabled */
#define SHT31_READSTATUS 0xF32D   /**< Read Out of Status Register */
#define SHT31_CLEARSTATUS 0x3041  /**< Clear Status */
#define SHT31_SOFTRESET 0x30A2    /**< Soft Reset */
#define SHT31_HEATEREN 0x306D     /**< Heater Enable */
#define SHT31_HEATERDIS 0x3066    /**< Heater Disable */
#define SHT31_REG_HEATER_BIT 0x0d /**< Status Register Heater Bit */

//extern TwoWire Wire; /**< Forward declarations of Wire for board/variant
                   //     combinations that don't have a default 'Wire' */

/**
 * Driver for the Adafruit SHT31-D Temperature and Humidity breakout board.
 */
  float readTemperature(void);
  float readHumidity(void);
  uint16_t readStatus(void);
  void reset(void);
  void heater(bool h);
  bool isHeaterEnabled();

  /**
   * Placeholder to track humidity internally.
   */


  bool readTempHum(void);
  HAL_StatusTypeDef writeCommand(uint16_t cmd);


#endif
