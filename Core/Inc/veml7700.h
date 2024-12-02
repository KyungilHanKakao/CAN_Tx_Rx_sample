// Copyright (c) Konstantin Belyalov. All rights reserved.
// Licensed under the MIT license.

#ifndef __VEML7700_H
#define __VEML7700_H

#include "main.h"

#ifdef __cplusplus
#define EXPORT extern "C"
#else
#define EXPORT
#endif

// ALS gain constants
#define REG_ALS_CONF_GAIN_1     (0x00 << 11) // x1 (default)
#define REG_ALS_CONF_GAIN_2     (0x01 << 11) // x2
#define REG_ALS_CONF_GAIN_1_8   (0x02 << 11) // x(1/8)
#define REG_ALS_CONF_GAIN_1_4   (0x03 << 11) // x(1/4)

// ALS integration times (ms)
#define REG_ALS_CONF_IT25       (0x0C << 6)
#define REG_ALS_CONF_IT50       (0x08 << 6)
#define REG_ALS_CONF_IT100      (0x00 << 6)
#define REG_ALS_CONF_IT200      (0x01 << 6)
#define REG_ALS_CONF_IT400      (0x02 << 6)
#define REG_ALS_CONF_IT800      (0x03 << 6)


// Define possible integration times (in milliseconds)
#define IT_25MS   25
#define IT_50MS   50
#define IT_100MS  100
#define IT_200MS  200
#define IT_400MS  400
#define IT_800MS  800

// Define possible gain values
#define GAIN_1X   1
#define GAIN_2X   2
#define GAIN_1_8X (1.0/8.0)
#define GAIN_1_4X (1.0/4.0)

enum {
  VEML7700_OK = 0,
  VEML7700_ERROR = 1,
};

typedef struct {
  I2C_HandleTypeDef *i2c;
  uint8_t            read_addr;
  uint8_t            write_addr;

  float              gain;
  uint16_t           integration_time;
  float			 	 resolution;  //lx/cnt
} veml7700;

// Initialize VEML7700 sensor.
// Params:
//  - `veml`: VEML7700 definition to be initialized
//  - `i2c` I2C HAL bus (`hi2c1`, `hi2c2`, etc)
//  - `addr` VEML7700 address. Either 0x10 or 0x48
// Returns:
//  - `VEML7700_OK` - device initialized successfully
//  - `VEML7700_ERROR` - initialization failed (e.g. device not found on the i2c bus)
EXPORT uint32_t veml7700_init(veml7700 *veml, I2C_HandleTypeDef *i2c, uint8_t addr);

// Power control //

// Power on veml7700
// Params:
//  - `veml` initialized instance
// Returns:
//  - `VEML7700_OK` - device powered on
//  - `VEML7700_ERROR` - I2C error
EXPORT uint32_t veml7700_power_on(veml7700 *veml);

// Shutdown veml7700 (make it sleep)
// Params:
//  - `veml` initialized instance
// Returns:
//  - `VEML7700_OK` - device put into sleep
//  - `VEML7700_ERROR` - I2C error
EXPORT uint32_t veml7700_shutdown(veml7700 *veml);

// ALS integration time configuration //

// Set Integration Time
// Params:
//  - `veml` initialized instance
//  - it - anything from REG_ALS_CONF_IT*
// Returns:
//  - `VEML7700_OK` - device put into sleep
//  - `VEML7700_ERROR` - I2C error
EXPORT uint32_t veml7700_set_als_integration_time(veml7700 *veml, uint16_t it);

// Get current Integration Time
// Params:
//  - `veml` initialized instance
// Returns current integration time
EXPORT uint16_t veml7700_get_als_integration_time(veml7700 *veml);

// ALS gain configuration //

// Set ALS gain
// Params:
//  - `veml` initialized instance
//  - gain - anything from REG_ALS_CONF_GAIN_*
// Returns:
//  - `VEML7700_OK` - device put into sleep
//  - `VEML7700_ERROR` - I2C error
EXPORT uint32_t veml7700_set_als_gain(veml7700 *veml, uint16_t gain);

// Get current ALS gain value
// Params:
//  - `veml` initialized instance
// Returns current gain value (REG_ALS_CONF_GAIN_*)
EXPORT uint16_t veml7700_get_als_gain(veml7700 *veml);

// Read current sensor values //

// Read previous measurement of ALS
// Params:
//  - `veml` initialized instance
// Returns current ALS
EXPORT uint16_t veml7700_read_als(veml7700 *veml);

// Read previous measurement of WHITE
// Params:
//  - `veml` initialized instance
// Returns current WHITE value
EXPORT uint16_t veml7700_read_white(veml7700 *veml);
EXPORT uint16_t veml7700_getID(veml7700 *veml);

#endif
