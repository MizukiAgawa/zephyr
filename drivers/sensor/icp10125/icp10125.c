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

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "ICP10125 driver enabled without any devices"
#endif

static inline struct icp10125_data *to_data(const struct device *dev)
{
	return dev->data;
}

static inline int icp10125_reg_read(uint8_t reg, uint8_t *data, uint16_t length,
		    struct icp10125_data *dev)
{
    return i2c_burst_read(dev->i2c, dev->i2c_addr, reg, data, length);
}

static inline int icp10125_reg_write(uint8_t reg, const uint8_t *data, uint16_t length,
		     struct icp10125_data *dev)
{
	return i2c_burst_write(dev->i2c, dev->i2c_addr, reg, data, length);
}

static int icp10125_reg_write_with_delay(uint8_t reg, const uint8_t *data, uint16_t length,
		     struct icp10125_data *dev, uint32_t delay_us)
{
	int ret = 0;

	ret = i2c_write(dev->i2c, data, length, reg);
	if (ret == 0) {
		k_usleep(delay_us);
	}
	return ret;
}
/*
 * Compensation code taken from ICP10125 datasheet, Section 4.2.3
 * "Compensation formula".
 */
static void icp10125_compensate_temp(struct icp10125_data *data, int32_t adc_temp)
{
	//int32_t var1, var2;

	// var1 = (((adc_temp >> 3) - ((int32_t)data->dig_t1 << 1)) *
	// 	((int32_t)data->dig_t2)) >> 11;
	// var2 = (((((adc_temp >> 4) - ((int32_t)data->dig_t1)) *
	// 	  ((adc_temp >> 4) - ((int32_t)data->dig_t1))) >> 12) *
	// 	((int32_t)data->dig_t3)) >> 14;

	// data->t_fine = var1 + var2;
	// data->comp_temp = (data->t_fine * 5 + 128) >> 8;
}

static int icp10125_wait_until_ready(const struct device *dev)
{
	uint8_t status = 0;
	int ret;

	/* Wait for NVM to copy and and measurement to be completed */
	do {
		k_sleep(K_MSEC(3));
		//ret = icp10125_reg_read(dev, ICP10125_REG_STATUS, &status, 1);
		ret = i2c_read(dev, &status, 1, ICP10125_REG_START_MEAS);
		printk("status:%d\n",status);
		if (ret < 0) {
			return ret;
		}
	} while (status == (157 || 168));

	return 0;
}

static int icp10125_sample_fetch(const struct device *dev,
			       enum sensor_channel chan)
{
	printk("fetch start\n");
	struct icp10125_data *data = dev->data;
	uint8_t data_write[5];
	uint8_t data_read[10] = {0};
	
	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);

	// data_write[0] = ICP10125_REG_SEND;
	// data_write[1] = 0x95;
	// data_write[2] = 0x00;
	// data_write[3] = 0x66;
	// data_write[4] = 0x9C;

	// printk("fetch data_write 1\n");
	// if (i2c_write(data->i2c, data_write, 5, ICP10125_I2C_ADDRESS)){
	// 	LOG_DBG("Failed to write address pointer");
	// 	return -EIO;
	// }

	// for (int i = 0; i < 4; i++) {
	// 	data_write[0] = 0xC7;
	// 	data_write[1] = 0xF7;
	// 	if (i2c_write(data->i2c, data_write, 2, ICP10125_I2C_ADDRESS)){
	// 		LOG_DBG("Failed to write address pointer");
	// 		return -EIO;
	// 	}

	// 	if (i2c_read(data->i2c, data_read, 3, ICP10125_I2C_ADDRESS)){
	// 		LOG_DBG("Failed to write address pointer");
	// 		return -EIO;
	// 	}
	//     data->otp[i] = data_read[0]<<8 | data_read[1]; 
	// }

	printk("fetch read temp\n");
	data_write[0] = 0x68;
	data_write[1] = 0x25;
	if (i2c_write(data->i2c, data_write, 2, ICP10125_I2C_ADDRESS)){
		LOG_DBG("Failed to write address pointer");
		return -EIO;
	}
	while(1){
		if (i2c_read(data->i2c, data_read, 3, ICP10125_I2C_ADDRESS)){
			continue;
		}
		break;
	}
	data->T_LSB = data_read[0]<<8 | data_read[1];
	printk("data[0]:%x\n,",data_read[0]);
	printk("data[1]:%x\n,",data_read[1]);
	printk("data[2]:%x\n,",data_read[2]);
	printk("data->T_LSB:%x\n,",data->T_LSB);

	printk("fetch end\n");

	return 0;
}

void init_base(struct icp10125_data * s, short *otp)
{ 
	int i;

	for(i = 0; i < 4; i++) s->sensor_constants[i] = (float)otp[i];
	s->p_Pa_calib[0] = 45000.0;
	s->p_Pa_calib[1] = 80000.0;
	s->p_Pa_calib[2] = 105000.0;
	s->LUT_lower = 3.5 * (1<<20);
	s->LUT_upper = 11.5 * (1<<20);
	s->quadr_factor = 1 / 16777216.0;
	s->offst_factor = 2048.0;
}

static void icp10125_cal_temp(uint16_t T_LSB, struct sensor_value *val){
	val->val1 = -45.f + 175.f/65536.f * T_LSB;
	val->val2 = ((-45.f + 175.f/65536.f * T_LSB) - val->val1) * 1000000;
}

static void icp10125_cal_press(uint16_t T_LSB, uint16_t p_LSB, struct sensor_value *val){
	float t;
	float s1,s2,s3;
	float in[3];
	float out[3];
	float A,B,C;
	// t = (float)(T_LSB - 32768);
	// s1 = data->LUT_lower + (float)(data->sensor_constants[0] * t * t) * data->quadr_factor;
	// s2 = data->offst_factor * data->sensor_constants[3] + (float)(data->sensor_constants[1] * t * t) * data->quadr_factor;
	// s3 = data->LUT_upper + (float)(data->sensor_constants[2] * t * t) * data->quadr_factor;
	in[0] = s1;
	in[1] = s2;
	in[2] = s3;
	//calculate_conversion_constants(s, s->p_Pa_calib, in, out);
	A = out[0];
	B = out[1];
	C = out[2];
	//*pressure = A + B / (C + p_LSB);
}

static int icp10125_channel_get(const struct device *dev,
                  enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct icp10125_data *data = dev->data;

	uint16_t p_LSB = 0;

	init_base(data, data->otp);

	if(chan == SENSOR_CHAN_AMBIENT_TEMP){
		icp10125_cal_temp(data->T_LSB, val);
	} else if(chan == SENSOR_CHAN_PRESS){
		//icp10125_cal_press();

		// t = (float)(T_LSB - 32768);
		// s1 = data->LUT_lower + (float)(data->sensor_constants[0] * t * t) * data->quadr_factor;
		// s2 = data->offst_factor * data->sensor_constants[3] + (float)(data->sensor_constants[1] * t * t) * data->quadr_factor;
		// s3 = data->LUT_upper + (float)(data->sensor_constants[2] * t * t) * data->quadr_factor;

		// in[0] = s1;
		// in[1] = s2;
		// in[2] = s3;
		// //calculate_conversion_constants(s, s->p_Pa_calib, in, out);
		// A = out[0];
		// B = out[1];
		// C = out[2];
		// //*pressure = A + B / (C + p_LSB);
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
	struct icp10125_data *drv_dev = to_data(dev);
	const struct icp10125_dev_config *cfg = dev->config;

	drv_dev->i2c = device_get_binding(cfg->i2c_master_name);
	if (drv_dev->i2c == NULL) {
		LOG_ERR("Could not get pointer to %s device",
			cfg->i2c_master_name);
		return -EINVAL;
	}

	drv_dev->i2c_addr = cfg->i2c_addr;

	/* Wait for the sensor to be ready */
	k_sleep(K_MSEC(1));

	LOG_DBG("\"%s\" OK", dev->name);
	printk("icp10125_init end\n");
	return 0;
}

/* Initializes a struct icp10125_config for an instance on an I2C bus. */
#define ICP10125_CONFIG_I2C(inst)			       \
	{					       \
		.bus.i2c = I2C_DT_SPEC_INST_GET(inst), \
		.bus_io = &icp10125_bus_io_i2c,	       \
	}

/*
 * Main instantiation macro, which selects the correct bus-specific
 * instantiation macros for the instance.
 */
#define ICP10125_DEFINE(inst)	           \
	static struct icp10125_data icp10125_drv_##inst;			\
	                                   \
	static const struct icp10125_dev_config icp10125_config_##inst =	{ \
		.i2c_master_name = DT_INST_BUS_LABEL(inst),	       \
		.i2c_addr = DT_INST_REG_ADDR(inst),		       \
	};                                 \
	                                   \ 
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
