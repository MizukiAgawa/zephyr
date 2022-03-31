/* icp10125.c - Driver for TDK ICP10125 temperature and pressure sensor */

/*
 * Copyright (c) 2022 Mizuki AGAWA <agawa.mizuki@fujitsu.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <drivers/i2c.h>
#include <kernel.h>
#include <drivers/sensor.h>
#include <init.h>
#include <sys/byteorder.h>
#include <sys/__assert.h>
#include <logging/log.h>

#ifdef CONFIG_ICP10125_CRC_CHECK
#include <sys/crc.h>
#endif /* CONFIG_ICP10125_CRC_CHECK */

#include "icp10125.h"

LOG_MODULE_REGISTER(ICP10125, CONFIG_SENSOR_LOG_LEVEL);

#define DT_DRV_COMPAT tdk_icp10125

static const uint8_t meas_addr_temp[4][2] = { { 0x60, 0x9C }, { 0x68, 0x25 }, { 0x70, 0xDF }, { 0x78, 0x66 } };
static const uint8_t meas_addr_press[4][2] = { { 0x40, 0x1A }, { 0x48, 0xA3 }, { 0x50, 0x59 }, { 0x59, 0xE0 } };
static const uint32_t conversion_time_max[4] = { 1800, 6300, 23800, 94500 };
static const uint32_t conversion_time_typ[4] = { 1600, 5600, 20800, 83200 };

/**
 * @brief The ICP10125 product manual does not have a detailed explanation,
 *        it was implemented in a format close to the sample code.
 */
static void icp10125_calculate_conversion_constants(float *p_LUT, float *A, float *B, float *C)
{
	const float p_Pa[] = { 45000.0, 80000.0, 105000.0 };

	*C = (p_LUT[0] * p_LUT[1] * (p_Pa[0] - p_Pa[1]) + p_LUT[1] * p_LUT[2] * (p_Pa[1] - p_Pa[2]) + p_LUT[2] * p_LUT[0] * (p_Pa[2] - p_Pa[0])) / (p_LUT[2] * (p_Pa[0] - p_Pa[1]) + p_LUT[0] * (p_Pa[1] - p_Pa[2]) + p_LUT[1] * (p_Pa[2] - p_Pa[0]));
	*A = (p_Pa[0] * p_LUT[0] - p_Pa[1] * p_LUT[1] - (p_Pa[1] - p_Pa[0]) * (*C)) / (p_LUT[0] - p_LUT[1]);
	*B = (p_Pa[0] - (*A)) * (p_LUT[0] + (*C));
}

/**
 * @brief The ICP10125 product manual does not have a detailed explanation,
 *        it was implemented in a format close to the sample code.
 */
static float icp10125_calc_caribrated_pressure(struct icp10125_data *data)
{
	const float quadr_factor = 1 / 16777216.0;
	const float offst_factor = 2048.0;
	const float LUT_lower = 3.5 * (1 << 20);
	const float LUT_upper = 11.5 * (1 << 20);
	float t;
	float in[3];
	float A, B, C;

	t = data->raw_temp_data - 32768.f;
	in[0] = LUT_lower + (data->sensor_constants[0] * t * t) * quadr_factor;
	in[1] = offst_factor * data->sensor_constants[3] + (data->sensor_constants[1] * t * t) * quadr_factor;
	in[2] = LUT_upper + (data->sensor_constants[2] * t * t) * quadr_factor;
	icp10125_calculate_conversion_constants(in, &A, &B, &C);
	return A + B / (C + data->raw_press_data);
}

/**
 * @brief Convert Pascal notation to Kilopascal notation.
 */
static void icp10125_convert_pressure_value(struct icp10125_data *data, struct sensor_value *val)
{
	float pressure = icp10125_calc_caribrated_pressure(data);

	val->val1 = (int32_t)(pressure / 100000);
	val->val2 = (int32_t)((pressure - (val->val1 * 100000)) * 10);
}

static void icp10125_convert_temperature_value(struct icp10125_data *data, struct sensor_value *val)
{
	float temp = -45.f + 175.f / 65536.f * data->raw_temp_data;

	val->val1 = (int32_t)temp;
	val->val2 = (int32_t)((temp - val->val1) * 1000000);
}

/**
 * @brief The ICP10125 product manual does not have a detailed explanation,
 *        it was implemented in a format close to the sample code.
 */
static int icp10125_read_otp(const struct device *dev)
{
	struct icp10125_data *data = dev->data;
	const struct icp10125_dev_config *cfg = dev->config;
	const uint8_t otp_read_setup_cmd[] = { 0xC5, 0x95, 0x00, 0x66, 0x9C };
	const uint8_t otp_read_request_cmd[] = { 0xC7, 0xF7 };
	uint8_t otp_data[3] = { 0 };

	if (i2c_write_dt(&cfg->i2c, otp_read_setup_cmd, sizeof(otp_read_setup_cmd))) {
		LOG_DBG("Failed to write otp_read_setup.");
		return -EIO;
	}

	for (int i = 0; i < 4; i++) {
		if (i2c_write_dt(&cfg->i2c, otp_read_request_cmd, sizeof(otp_read_request_cmd))) {
			LOG_DBG("Failed to write otp_read_request.");
			return -EIO;
		}

		if (i2c_read_dt(&cfg->i2c, otp_data, sizeof(otp_data))) {
			LOG_DBG("Failed to read otp_read_request.");
			return -EIO;
		}
		data->sensor_constants[i] = (float)(otp_data[0] << 8 | otp_data[1]);
	}
	return 0;
}

static int icp10125_sample_fetch(const struct device *dev,
				 enum sensor_channel chan)
{
	struct icp10125_data *data = dev->data;
	const struct icp10125_dev_config *cfg = dev->config;

	if (chan == SENSOR_CHAN_AMBIENT_TEMP || chan == SENSOR_CHAN_ALL) {
		if (i2c_write_dt(&cfg->i2c, meas_addr_temp[cfg->op_mode_temperature], 2)) {
			LOG_DBG("Failed to measure the temperature.");
			return -EIO;
		}

		k_sleep(K_USEC(conversion_time_typ[cfg->op_mode_temperature]));
		if (i2c_read_dt(&cfg->i2c, data->read_data, 3)) {
			k_sleep(K_USEC(conversion_time_max[cfg->op_mode_temperature] - conversion_time_typ[cfg->op_mode_temperature]));
			if (i2c_read_dt(&cfg->i2c, data->read_data, 3)) {
				return -EIO;
			}
		}
#ifdef CONFIG_ICP10125_CRC_CHECK
	/* Calculate CRC from Chapter 5 Section 8 of ICP10125 Product manuals. */
	if (crc8(data->read_data, 2, 0x31, 0xFF, false) != data->read_data[2]) {
		LOG_ERR("CRC verification failed.");
		return -EIO;
	}
#endif /* CONFIG_ICP10125_CRC_CHECK */
		data->raw_temp_data = data->read_data[0] << 8 | data->read_data[1];
	}

	if (chan == SENSOR_CHAN_PRESS || chan == SENSOR_CHAN_ALL) {
		if (i2c_write_dt(&cfg->i2c, meas_addr_press[cfg->op_mode_pressure], 2)) {
			LOG_DBG("Failed to measure the pressure.");
			return -EIO;
		}
		k_sleep(K_USEC(conversion_time_typ[cfg->op_mode_pressure]));
		if (i2c_read_dt(&cfg->i2c, data->read_data, 9)) {
			k_sleep(K_USEC(conversion_time_max[cfg->op_mode_pressure] - conversion_time_typ[cfg->op_mode_pressure]));
			if (i2c_read_dt(&cfg->i2c, data->read_data, 9)) {
				return -EIO;
			}
		}
#ifdef CONFIG_ICP10125_CRC_CHECK
		/* Calculate CRC from Chapter 5 Section 8 of ICP10125 Product manuals. */
		for(int i = 0; i < 3; i++){
			if (crc8(&data->read_data[i * 3], 2, 0x31, 0xFF, false) != data->read_data[3 * i + 2]) {
				LOG_ERR("CRC verification failed.");
				return -EIO;
			}
		}

#endif /* CONFIG_ICP10125_CRC_CHECK */
		data->raw_press_data = data->read_data[0] << 16 | data->read_data[1] << 8 | data->read_data[3];
		data->raw_temp_data = data->read_data[6] << 8 | data->read_data[7];
	}
	return 0;
}

static int icp10125_channel_get(const struct device *dev,
				enum sensor_channel chan,
				struct sensor_value *val)
{
	struct icp10125_data *data = dev->data;

	if (!(chan == SENSOR_CHAN_AMBIENT_TEMP || chan == SENSOR_CHAN_PRESS || chan == SENSOR_CHAN_ALL)) {
		return -ENOTSUP;
	}
	if (chan == SENSOR_CHAN_AMBIENT_TEMP || chan == SENSOR_CHAN_ALL) {
		icp10125_convert_temperature_value(data, val);
	}
	if (chan == SENSOR_CHAN_PRESS || chan == SENSOR_CHAN_ALL) {
		icp10125_convert_pressure_value(data, val);
	}
	return 0;
}

static const struct sensor_driver_api icp10125_api_funcs = {
	.sample_fetch = icp10125_sample_fetch,
	.channel_get = icp10125_channel_get,
};

static int icp10125_init(const struct device *dev)
{
	if (icp10125_read_otp(dev)) {
		return -EIO;
	};
	return 0;
}

#define ICP10125_DEFINE(inst)							  \
	static struct icp10125_data icp10125_drv_##inst;			  \
	static const struct icp10125_dev_config icp10125_config_##inst =        { \
		.i2c = I2C_DT_SPEC_INST_GET(inst),				  \
		.op_mode_temperature = DT_ENUM_IDX(DT_DRV_INST(inst), op_mode_temperature),		  \
		.op_mode_pressure = DT_ENUM_IDX(DT_DRV_INST(inst), op_mode_pressure) };	  \
	DEVICE_DT_INST_DEFINE(inst,						  \
			      icp10125_init,					  \
			      NULL,						  \
			      &icp10125_drv_##inst,				  \
			      &icp10125_config_##inst,				  \
			      POST_KERNEL,					  \
			      CONFIG_SENSOR_INIT_PRIORITY,			  \
			      &icp10125_api_funcs);

DT_INST_FOREACH_STATUS_OKAY(ICP10125_DEFINE)
