/*
 * Copyright (c) 2022 Mizuki AGAWA <agawa.mizuki@fujitsu.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_ICP10125_ICP10125_H_
#define ZEPHYR_DRIVERS_SENSOR_ICP10125_ICP10125_H_

#include <zephyr/types.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/i2c.h>

struct icp10125_data {
	/* Compensated values. */
	const struct device *i2c;
	int16_t otp[4];
	uint8_t i2c_addr;
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
	uint8_t op_mode_t;
	uint8_t op_mode_p;
};

#endif /* ZEPHYR_DRIVERS_SENSOR_ICP10125_ICP10125_H_ */
