/* icp10125.c - Driver for Bosch ICP10125 temperature and pressure sensor */

/*
 * Copyright (c) 2016, 2017 Intel Corporation
 * Copyright (c) 2017 IpTronix S.r.l.
 * Copyright (c) 2021 Nordic Semiconductor ASA
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

#include <drivers/gpio.h>
#include <drivers/i2c.h>
#include "icp10125.h"

LOG_MODULE_REGISTER(ICP10125, CONFIG_SENSOR_LOG_LEVEL);



#define DT_DRV_COMPAT tdk_icp10125
#define ICP10125_BUS_I2C DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
#define ICP10125_I2C_ADDRESS              DT_INST_REG_ADDR(0)


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




#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "ICP10125 driver enabled without any devices"
#endif

int send_soft_reset(const struct device *dev)
{
	struct icp10125_data *data = dev->data;
	uint8_t data_write[5];
	
	data_write[0] = 0x80;
	data_write[1] = 0x5D;

	if (i2c_write(data->i2c, data_write, 2, ICP10125_I2C_ADDRESS)){
		LOG_DBG("Failed to write address pointer");
		return -EIO;
	}
	return 0;
}

void init_base(struct icp10125_data *s)
{ 
	for(int i = 0; i < 4; i++)
	{
		s->sensor_constants[i] = (float)s->otp[i];
	};
	s->p_Pa_calib[0] = 45000.0;
	s->p_Pa_calib[1] = 80000.0;
	s->p_Pa_calib[2] = 105000.0;
	s->LUT_lower = 3.5 * (1<<20);
	s->LUT_upper = 11.5 * (1<<20);
	s->quadr_factor = 1 / 16777216.0;
	s->offst_factor = 2048.0;
}

void calculate_conversion_constants(float *p_Pa, float *p_LUT, float *out)
{
	float A,B,C;
	C = (p_LUT[0] * p_LUT[1] * (p_Pa[0] - p_Pa[1]) + p_LUT[1] * p_LUT[2] * (p_Pa[1] - p_Pa[2]) + p_LUT[2] * p_LUT[0] * (p_Pa[2] - p_Pa[0])) / (p_LUT[2] * (p_Pa[0] - p_Pa[1]) + p_LUT[0] * (p_Pa[1] - p_Pa[2]) + p_LUT[1] * (p_Pa[2] - p_Pa[0]));
	A = (p_Pa[0] * p_LUT[0] - p_Pa[1] * p_LUT[1] - (p_Pa[1] - p_Pa[0]) * C) / (p_LUT[0] - p_LUT[1]);
	B = (p_Pa[0] - A) * (p_LUT[0] + C);
	out[0] = A;
	out[1] = B;
	out[2] = C;
}

static void icp10125_cal_temp(uint16_t T_LSB, struct sensor_value *val){
	val->val1 = -45.f + 175.f/65536.f * T_LSB;
	val->val2 = ((-45.f + 175.f/65536.f * T_LSB) - val->val1) * 1000000;
}

static void icp10125_cal_press(struct icp10125_data *data, struct sensor_value *val){
	float t;
	float s1,s2,s3;
	float in[3];
	float out[3];
	float A,B,C;
	float pessure;
	t = (float)(data->T_LSB - 32768);
	s1 = data->LUT_lower + (float)(data->sensor_constants[0] * t * t) * data->quadr_factor;
	s2 = data->offst_factor * data->sensor_constants[3] + (float)(data->sensor_constants[1] * t * t) * data->quadr_factor;
	s3 = data->LUT_upper + (float)(data->sensor_constants[2] * t * t) * data->quadr_factor;
	in[0] = s1;
	in[1] = s2;
	in[2] = s3;
	calculate_conversion_constants(data->p_Pa_calib, in, out);
	A = out[0];
	B = out[1];
	C = out[2];
	pessure = A + B / (C + data->p_LSB);
	val->val1 = (int32_t)(A + B / (C + data->p_LSB));
	val->val2 = (int32_t)((A + B / (C + data->p_LSB) - val->val1) * 1000000);
}

int read_otp(const struct device *dev)
{
	struct icp10125_data *data = dev->data;
	uint8_t data_write[5];
	uint8_t data_read[10] = {0};
	
	data_write[0] = CALIBRATION_PARAME_SEND_01;
	data_write[1] = CALIBRATION_PARAME_SEND_02;
	data_write[2] = CALIBRATION_PARAME_SEND_03;
	data_write[3] = CALIBRATION_PARAME_SEND_04;
	data_write[4] = CALIBRATION_PARAME_SEND_05;

	if (i2c_write(data->i2c, data_write, 5, ICP10125_I2C_ADDRESS)){
		LOG_DBG("Failed to write address pointer");
		return -EIO;
	}

	for (int i = 0; i < 4; i++) {
		data_write[0] = CALIBRATION_PARAME_READ_01;
		data_write[1] = CALIBRATION_PARAME_READ_02;
		if (i2c_write(data->i2c, data_write, 2, ICP10125_I2C_ADDRESS)){
			LOG_DBG("Failed to write address pointer");
			return -EIO;
		}

		if (i2c_read(data->i2c, data_read, 3, ICP10125_I2C_ADDRESS)){
			LOG_DBG("Failed to write address pointer");
			return -EIO;
		}
	    data->otp[i] = data_read[0] << 8 | data_read[1]; 
	}
	init_base(data);
	return 0;
}

static int icp10125_sample_fetch(const struct device *dev,
			       enum sensor_channel chan)
{
	struct icp10125_data *data = dev->data;
	uint8_t data_write[4];
	uint8_t data_read[9] = {0};
	
	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);

	if(chan == SENSOR_CHAN_AMBIENT_TEMP || chan == SENSOR_CHAN_ALL){
		data_write[0] = MEAS_ADDR_T_H;
		data_write[1] = MEAS_ADDR_T_L;
		if (i2c_write(data->i2c, data_write, 2, ICP10125_I2C_ADDRESS)){
			LOG_DBG("Failed to write address pointer");
			return -EIO;
		}
		while(1){
			int count = 0;
			if(count >= 2){
				return -EIO;
			}
			if (i2c_read(data->i2c, data_read, 3, ICP10125_I2C_ADDRESS)){
				count++;
				k_sleep(K_USEC(CONVERSION_TIME_T));
				continue;
			}
			break;
		}
		data->T_LSB = data_read[0] << 8 | data_read[1];
	}

	if(chan == SENSOR_CHAN_PRESS || chan == SENSOR_CHAN_ALL){
		data_write[0] = MEAS_ADDR_P_H;
		data_write[1] = MEAS_ADDR_P_L;
		if (i2c_write(data->i2c, data_write, 2, ICP10125_I2C_ADDRESS)){
			LOG_DBG("Failed to write address pointer");
			return -EIO;
		}
		while(1){
			int count = 0;
			if(count >= 2){
				return -EIO;
			}
			if (i2c_read(data->i2c, data_read, 9, ICP10125_I2C_ADDRESS)){
				count++;
				k_sleep(K_USEC(CONVERSION_TIME_P));
				continue;
			}
			break;
		}
		data->p_LSB = (int32_t)(data_read[0] << 16 | data_read[1] << 8 | data_read[3]);
		data->T_LSB = data_read[6] << 8 | data_read[7];
	}

	return 0;
}

static int icp10125_channel_get(const struct device *dev,
                  enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct icp10125_data *data = dev->data;

	if(chan == SENSOR_CHAN_AMBIENT_TEMP){
		icp10125_cal_temp(data->T_LSB, val);
	} else if(chan == SENSOR_CHAN_PRESS){
		icp10125_cal_press(data, val);
	} else {
		return -ENOTSUP;
	}
	return 0;
}

static const struct sensor_driver_api icp10125_api_funcs = {
	.sample_fetch = icp10125_sample_fetch,
	.channel_get = icp10125_channel_get,
};

static int icp10125_init(const struct device *dev)
{
	struct icp10125_data *drv_dev = dev->data;
	const struct icp10125_dev_config *cfg = dev->config;

	drv_dev->i2c = device_get_binding(cfg->i2c_master_name);
	if (drv_dev->i2c == NULL) {
		LOG_ERR("Could not get pointer to %s device",
			cfg->i2c_master_name);
		return -EINVAL;
	}

	drv_dev->i2c_addr = cfg->i2c_addr;

	//soft reset
	if(send_soft_reset(dev))
	{
		LOG_DBG("Failed send_soft_reset error");
		return -EIO;
	};

	/* Wait for the sensor to be ready */
	k_sleep(K_MSEC(1));

	if(read_otp(dev))
	{
		LOG_DBG("Failed read_otp error");
		return -EIO;
	};

	LOG_DBG("\"%s\" OK", dev->name);
	return 0;
}

/*
 * Main instantiation macro, which selects the correct bus-specific
 * instantiation macros for the instance.
 */
#define ICP10125_DEFINE(inst)	           \
	static struct icp10125_data icp10125_drv_##inst;			\
	static const struct icp10125_dev_config icp10125_config_##inst =	{ \
		.i2c_master_name = DT_INST_BUS_LABEL(inst),	       \
		.i2c_addr = DT_INST_REG_ADDR(inst)};		       \
	DEVICE_DT_INST_DEFINE(inst,					\
			 icp10125_init,				\
			 NULL,				\
			 &icp10125_drv_##inst,				\
			 &icp10125_config_##inst,		       \
			 POST_KERNEL,					\
			 CONFIG_SENSOR_INIT_PRIORITY,			\
			 &icp10125_api_funcs);

/* Create the struct device for every status "okay" node in the devicetree. */
DT_INST_FOREACH_STATUS_OKAY(ICP10125_DEFINE)
