/* pim586.c - Driver for Bosch PIM586 temperature and pressure sensor */

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
#include "pim586.h"

LOG_MODULE_REGISTER(PIM586, CONFIG_SENSOR_LOG_LEVEL);

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "PIM586 driver enabled without any devices"
#endif

static inline struct pim586_data *to_data(const struct device *dev)
{
	return dev->data;
}

static inline int pim586_reg_read(uint8_t reg, uint8_t *data, uint16_t length,
		    struct pim586_data *dev)
{
    return i2c_burst_read(dev->i2c, dev->i2c_addr, reg, data, length);
}

static inline int pim586_reg_write(uint8_t reg, const uint8_t *data, uint16_t length,
		     struct pim586_data *dev)
{
	return i2c_burst_write(dev->i2c, dev->i2c_addr, reg, data, length);
}

static int pim586_reg_write_with_delay(uint8_t reg, const uint8_t *data, uint16_t length,
		     struct pim586_data *dev, uint32_t delay_us)
{
	int ret = 0;

	ret = reg_write(reg, data, length, dev);
	if (ret == 0) {
		k_usleep(delay_us);
	}
	return ret;
}
/*
 * Compensation code taken from PIM586 datasheet, Section 4.2.3
 * "Compensation formula".
 */
static void pim586_compensate_temp(struct pim586_data *data, int32_t adc_temp)
{
	int32_t var1, var2;

	// var1 = (((adc_temp >> 3) - ((int32_t)data->dig_t1 << 1)) *
	// 	((int32_t)data->dig_t2)) >> 11;
	// var2 = (((((adc_temp >> 4) - ((int32_t)data->dig_t1)) *
	// 	  ((adc_temp >> 4) - ((int32_t)data->dig_t1))) >> 12) *
	// 	((int32_t)data->dig_t3)) >> 14;

	// data->t_fine = var1 + var2;
	// data->comp_temp = (data->t_fine * 5 + 128) >> 8;
}

static int pim586_wait_until_ready(const struct device *dev)
{
	uint8_t status = 0;
	int ret;

	/* Wait for NVM to copy and and measurement to be completed */
	do {
		k_sleep(K_MSEC(3));
		//ret = pim586_reg_read(dev, PIM586_REG_STATUS, &status, 1);
		ret = pim586_reg_read(dev, PIM586_REG_START_MEAS, &status, 1);
		printk("status:%d\n",status);
		if (ret < 0) {
			return ret;
		}
	} while (status == (157 || 168));

	return 0;
}

static int pim586_sample_fetch(const struct device *dev,
			       enum sensor_channel chan)
{
	printk("fetch start\n");
	struct pim586_data *data = to_data(dev);
	uint8_t buf[5];
	int32_t adc_press, adc_temp, adc_humidity;
	int size = 2;
	int ret;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);

	buf[0] = PIM586_REG_SEND;
	buf[1] = 0x95;
	buf[2] = 0x00;
	buf[3] = 0x66;
	buf[4] = 0x9C;

	printk("fetch data_write 1\n");

	if (i2c_write(data->i2c, buf, 5, 0x63) ){
		LOG_DBG("Failed to write address pointer");
		return -EIO;
	}

	// ret = pim586_reg_write(dev, PIM586_REG_WRITE_HEAD, data_write);
	// if (ret < 0) {
	// 	printk("fetch data_write 2\n");
	// 	LOG_DBG("CONFIG write failed: %d", ret);
	// 	return ret;
	// }


	printk("fetch end\n");
	return 0;
}

static int pim586_channel_get(const struct device *dev,
			      enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct pim586_data *data = to_data(dev);

	// switch (chan) {
	// case SENSOR_CHAN_AMBIENT_TEMP:
	// 	/*
	// 	 * data->comp_temp has a resolution of 0.01 degC.  So
	// 	 * 5123 equals 51.23 degC.
	// 	 */
	// 	val->val1 = data->comp_temp / 100;
	// 	val->val2 = data->comp_temp % 100 * 10000;
	// 	break;
	// default:
	// 	return -EINVAL;
	// }

	return 0;
}

static const struct sensor_driver_api pim586_api_funcs = {
	.sample_fetch = pim586_sample_fetch,
	.channel_get = pim586_channel_get,
};


static int pim586_init(const struct device *dev)
{
	struct pim586_data *drv_dev = to_data(dev);
	const struct pim586_dev_config *cfg = dev->config;
	int err;

	drv_dev->i2c = device_get_binding(cfg->i2c_master_name);
	if (drv_dev->i2c == NULL) {
		LOG_ERR("Could not get pointer to %s device",
			cfg->i2c_master_name);
		return -EINVAL;
	}

	drv_dev->i2c_addr = cfg->i2c_addr;
	drv_dev->acc_odr = PIM586_ACC_ODR_100_HZ;
	drv_dev->acc_range = 8;
	drv_dev->gyr_odr = PIM586_GYR_ODR_200_HZ;
	drv_dev->gyr_range = 2000;

	// k_usleep(BMI270_POWER_ON_TIME);

	// printk("pim586_init start\n");
	// err = pim586_bus_check(dev);
	// if (err < 0) {
	// 	LOG_DBG("bus check failed: %d", err);
	// 	return err;
	// }
	// printk("pim586_init 2\n");

	/* Wait for the sensor to be ready */
	k_sleep(K_MSEC(1));

	LOG_DBG("\"%s\" OK", dev->name);
	printk("pim586_init end\n");
	return 0;
}

/* Initializes a struct pim586_config for an instance on an I2C bus. */
#define PIM586_CONFIG_I2C(inst)			       \
	{					       \
		.bus.i2c = I2C_DT_SPEC_INST_GET(inst), \
		.bus_io = &pim586_bus_io_i2c,	       \
	}

/*
 * Main instantiation macro, which selects the correct bus-specific
 * instantiation macros for the instance.
 */
#define PIM586_DEFINE(inst)	           \
	static struct pim586_data pim586_drv_##inst;			\
	                                   \
	static const struct pim586_dev_config pim586_config_##inst =	{ \
		.i2c_master_name = DT_INST_BUS_LABEL(inst),	       \
		.i2c_addr = DT_INST_REG_ADDR(inst),		       \
	};                                 \
	                                   \ 
	DEVICE_DT_INST_DEFINE(inst,					\
			 pim586_init,				\
			 NULL,				\
			 &pim586_drv_##inst,				\
			 &pim586_config_##inst,		       \
			 POST_KERNEL,					\
			 CONFIG_SENSOR_INIT_PRIORITY,			\
			 &pim586_api_funcs);

/* Create the struct device for every status "okay" node in the devicetree. */
DT_INST_FOREACH_STATUS_OKAY(PIM586_DEFINE)
