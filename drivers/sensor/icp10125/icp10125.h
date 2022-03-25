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
	uint16_t raw_temp_data;
	uint32_t raw_press_data;
	uint8_t read_data[9];

	float sensor_constants[4];
};

struct icp10125_dev_config {
	struct i2c_dt_spec i2c;
	uint8_t op_mode_t;
	uint8_t op_mode_p;
};

#endif /* ZEPHYR_DRIVERS_SENSOR_ICP10125_ICP10125_H_ */
