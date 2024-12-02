// Copyright (c) Konstantin Belyalov. All rights reserved.
// Licensed under the MIT license.

#include <stdio.h>
#include "veml7700.h"

// VEML7700 registers //
#define REG_ALS_CONF            0x00
#define REG_ALS_WH              0x01
#define REG_ALS_WL              0x02
#define REG_POWER_SAVING        0x03
#define REG_ALS                 0x04
#define REG_WHITE               0x05
#define REG_ALS_INT             0x06
#define REG_ID		            0x07

// Register 0x0: ALS_CONF //
// ALS integration times - all bits
#define REG_ALS_CONF_IT_CLEAR   (0x0f << 6)
// ALS persistent protect number
#define REG_ALS_CONF_PERS_25    (0xC0 << 4)  //25ms
#define REG_ALS_CONF_PERS_50    (0x80 << 4)  //50ms
#define REG_ALS_CONF_PERS_1     (0x00 << 4)  //100ms
#define REG_ALS_CONF_PERS_2     (0x01 << 4)  //200ms
#define REG_ALS_CONF_PERS_4     (0x02 << 4)	 //400ms
#define REG_ALS_CONF_PERS_8     (0x03 << 4)  //800ms
// ALS interrupt enable
#define REG_ALS_CONF_IT_ENABLE  (0x01 << 1)
// ALS shutdown setting
#define REG_ALS_CONF_SHUTDOWN   0x01

// Register 0x3: POWER SAVING
// Power saving modes
#define REG_POWER_SAVING_PSM_1  (0x00 << 1)
#define REG_POWER_SAVING_PSM_2  (0x01 << 1)
#define REG_POWER_SAVING_PSM_3  (0x02 << 1)
#define REG_POWER_SAVING_PSM_4  (0x03 << 1)
#define REG_POWER_SAVING_ENABLE  0x01

static uint32_t _write_register(veml7700 *veml, uint8_t reg, uint16_t value)
{
  uint8_t payload[3] = {reg, value & 0xff, value >> 8};

  return HAL_I2C_Master_Transmit(veml->i2c, veml->write_addr, &payload[0], 3, 500);
}

static uint16_t _read_register(veml7700 *veml, uint8_t reg)
{
  uint8_t payload[2] = {0};

  int err = HAL_I2C_Mem_Read(veml->i2c, veml->write_addr, reg, 1, &payload[0], 2, 500);
  if (err != HAL_OK) {
    return 0;
  }
  return (payload[1] << 8) | payload[0];
}

uint32_t veml7700_init(veml7700 *veml, I2C_HandleTypeDef *i2c, uint8_t addr)
{
  veml->read_addr  = (addr << 1) | 0x01;
  veml->write_addr = (addr << 1);
  veml->i2c        = i2c;
  veml->gain = 0.0336;

  // Reset VEML configuration (in order to check device)
  return _write_register(veml, REG_ALS_CONF, 0);
}

uint32_t veml7700_power_on(veml7700 *veml)
{
  // Get current config and clear shutdown bit
  uint16_t config = _read_register(veml, REG_ALS_CONF);
  config &= ~REG_ALS_CONF_SHUTDOWN;

  return _write_register(veml, REG_ALS_CONF, config);
}

uint32_t veml7700_shutdown(veml7700 *veml)
{
  // Get current config and set shutdown bit
  uint16_t config = _read_register(veml, REG_ALS_CONF);
  config |= REG_ALS_CONF_SHUTDOWN;

  return _write_register(veml, REG_ALS_CONF, config);
}

double calculateResolution(veml7700 *veml) {

	uint16_t integrationTime=veml->integration_time;
	double gain=veml->gain;
    double baseResolution = 0.0042; // Base resolution for IT = 800ms and Gain = 2x

    // Adjust base resolution according to integration time
    switch (integrationTime) {
        case IT_25MS:
            baseResolution *= (800 / 25);
            break;
        case IT_50MS:
            baseResolution *= (800 / 50);
            break;
        case IT_100MS:
            baseResolution *= (800 / 100);
            break;
        case IT_200MS:
            baseResolution *= (800 / 200);
            break;
        case IT_400MS:
            baseResolution *= (800 / 400);
            break;
        case IT_800MS:
            // No change needed, as this is the base case
            break;
        default:
            printf("Invalid integration time\n");
            return -1;
    }

    // Adjust resolution according to gain setting
    if (gain == GAIN_1X) {
        baseResolution *= (2 / GAIN_1X);
    } else if (gain == GAIN_2X) {
        // No change needed, as this is the base case
    } else if (gain == GAIN_1_4X) {
        baseResolution *= (2 / GAIN_1_4X);
    } else if (gain == GAIN_1_8X) {
        baseResolution *= (2 / GAIN_1_8X);
    } else {
        printf("Invalid gain\n");
        return -1;
    }

    veml->resolution=baseResolution;
    return baseResolution;
}


void veml7700_set_integration_time_for_resolution(veml7700 *veml, uint16_t it)
{
	switch(it){
	case REG_ALS_CONF_IT25:
		veml->integration_time=25;
		break;
	case REG_ALS_CONF_IT50:
		veml->integration_time=50;
		break;
	case REG_ALS_CONF_IT100:
		veml->integration_time=100;
		break;
	case REG_ALS_CONF_IT200:
		veml->integration_time=200;
		break;
	case REG_ALS_CONF_IT400:
		veml->integration_time=400;
		break;
	case REG_ALS_CONF_IT800:
		veml->integration_time=800;
		break;
	}
}


uint32_t veml7700_set_als_integration_time(veml7700 *veml, uint16_t it)
{
  uint16_t config = _read_register(veml, REG_ALS_CONF);
  config &= ~REG_ALS_CONF_IT_CLEAR;
  config |= it;

  veml7700_set_integration_time_for_resolution(veml, it);
  calculateResolution(veml);
  return _write_register(veml, REG_ALS_CONF, config);
}

uint16_t veml7700_get_als_integration_time(veml7700 *veml)
{
  uint16_t config = _read_register(veml, REG_ALS_CONF);
  return (config & REG_ALS_CONF_IT_CLEAR) >> 6;
}

void veml7700_set_gain_for_resolution(veml7700 *veml, uint16_t gain)
{
	switch(gain){
	case REG_ALS_CONF_GAIN_1:
		veml->gain=1;
		break;
	case REG_ALS_CONF_GAIN_2:
		veml->gain=2;
		break;
	case REG_ALS_CONF_GAIN_1_8:
		veml->gain=(1.0/8.0);
		break;
	case REG_ALS_CONF_GAIN_1_4:
		veml->gain=(1.0/4.0);
		break;
	}

}

uint32_t veml7700_set_als_gain(veml7700 *veml, uint16_t gain)
{
  uint16_t config = _read_register(veml, REG_ALS_CONF);
  // Clear all gain bits
  config &= ~REG_ALS_CONF_GAIN_1_4;
  config |= gain;
  veml7700_set_gain_for_resolution( veml,  gain);
  calculateResolution(veml);
  return _write_register(veml, REG_ALS_CONF, gain);
}


uint16_t veml7700_get_als_gain(veml7700 *veml)
{
  uint16_t config = _read_register(veml, REG_ALS_CONF);
  return (config & REG_ALS_CONF_GAIN_1_4) >> 11;
}

uint16_t veml7700_read_als(veml7700 *veml)
{
  return _read_register(veml, REG_ALS);
}

uint16_t veml7700_read_white(veml7700 *veml)
{
  return _read_register(veml, REG_WHITE);
}
uint16_t veml7700_getID(veml7700 *veml)
{
  return _read_register(veml, REG_ID);
}


