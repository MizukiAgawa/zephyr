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

#define MODE_LP_T
#define MODE_LP_P

#ifdef MODE_LP_T
#define CONVERSION_TIME_T 1800
#define MEAS_ADDR_T_H 0x60
#define MEAS_ADDR_T_L 0x9C
#endif
#ifdef MODE_N_T
#define CONVERSION_TIME_T 6300
#define MEAS_ADDR_T_H 0x68
#define MEAS_ADDR_T_L 0x25
#endif
#ifdef MODE_LN_T
#define CONVERSION_TIME_T 23800
#define MEAS_ADDR_T_H 0x70
#define MEAS_ADDR_T_L 0xDF
#endif
#ifdef MODE_ULN_T
#define CONVERSION_TIME_T 94500
#define MEAS_ADDR_T_H 0x78
#define MEAS_ADDR_T_L 0x66
#endif

#ifdef MODE_LP_P
#define CONVERSION_TIME_P 1800
#define MEAS_ADDR_P_H 0x40
#define MEAS_ADDR_P_L 0x1A
#endif
#ifdef MODE_N_P
#define CONVERSION_TIME_P 6300
#define MEAS_ADDR_P_H 0x48
#define MEAS_ADDR_P_L 0xA3
#endif
#ifdef MODE_LN_P
#define CONVERSION_TIME_P 23800
#define MEAS_ADDR_P_H 0x50
#define MEAS_ADDR_P_L 0x59
#endif
#ifdef MODE_ULN_P
#define CONVERSION_TIME_P 94500
#define MEAS_ADDR_P_H 0x58
#define MEAS_ADDR_P_L 0xE0
#endif

#define CALIBRATION_PARAME_SEND_01 0xC5
#define CALIBRATION_PARAME_SEND_02 0x95
#define CALIBRATION_PARAME_SEND_03 0x00
#define CALIBRATION_PARAME_SEND_04 0x66
#define CALIBRATION_PARAME_SEND_05 0x9C

#define CALIBRATION_PARAME_READ_01 0xC7
#define CALIBRATION_PARAME_READ_02 0xF7

#define ICP10125_I2C_ADDRESS              DT_INST_REG_ADDR(0)

#endif /* ZEPHYR_DRIVERS_SENSOR_ICP10125_ICP10125_H_ */
