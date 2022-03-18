/*
 * Copyright (c) 2016, 2017 Intel Corporation
 * Copyright (c) 2017 IpTronix S.r.l.
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_ICP10125_ICP10125_H_
#define ZEPHYR_DRIVERS_SENSOR_ICP10125_ICP10125_H_

#include <zephyr/types.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/i2c.h>

#define DT_DRV_COMPAT tdk_icp10125

#define ICP10125_BUS_I2C DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)

struct icp10125_data {
	/* Compensated values. */
	const struct device *i2c;
	int16_t otp[4];
	uint8_t i2c_addr;
	uint32_t min_delay_us;
	uint8_t pressure_en;
	uint8_t temperature_en;
	float sensor_constants[4]; // OTP values
	float p_Pa_calib[3];
	float LUT_lower;
	float LUT_upper;
	float quadr_factor;
	float offst_factor;
	int32_t T_LSB;
	int32_t p_LSB;
};

struct icp10125_dev_config {
	const char *i2c_master_name;
	uint16_t i2c_addr;
};

#define ICP10125_I2C_ADDRESS              DT_INST_REG_ADDR(0)
#define ICP10125_REG_SEND                 0xC5

#define ICP10125_REG_PRESS_MSB            0xF7
#define ICP10125_REG_COMP_START           0x88
#define ICP10125_REG_HUM_COMP_PART1       0xA1
#define ICP10125_REG_HUM_COMP_PART2       0xE1
#define ICP10125_REG_ID                   0xD0
#define ICP10125_REG_CONFIG               0xF5
#define ICP10125_REG_CTRL_MEAS            0xF4
#define ICP10125_REG_CTRL_HUM             0xF2
#define ICP10125_REG_STATUS               0xF3
//#define ICP10125_REG_RESET                0xE0
#define ICP10125_REG_WRITE_HEAD           0xC6
#define ICP10125_REG_RESET                0x80
#define ICP10125_REG_START_MEAS           0xC7
#define ICP10125_REG_START_MEAS_READ      0xF7
#define ICP10125_REG_START_MEAS_RETURN    0x9D

#define BMP280_CHIP_ID_SAMPLE_1         0x56
#define BMP280_CHIP_ID_SAMPLE_2         0x57
#define BMP280_CHIP_ID_MP               0x58
#define ICP10125_CHIP_ID                  0x60
#define ICP10125_MODE_SLEEP               0x00
#define ICP10125_MODE_FORCED              0x01
#define ICP10125_MODE_NORMAL              0x03
#define ICP10125_SPI_3W_DISABLE           0x00
//#define ICP10125_CMD_SOFT_RESET           0xB6
#define ICP10125_CMD_SOFT_RESET           0x5D
#define ICP10125_STATUS_MEASURING         0x08
#define ICP10125_STATUS_IM_UPDATE         0x01

#endif /* ZEPHYR_DRIVERS_SENSOR_ICP10125_ICP10125_H_ */
