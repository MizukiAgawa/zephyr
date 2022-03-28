/*
 * Copyright (c) 2022 Mizuki AGAWA <agawa.mizuki@fujitsu.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/sensor.h>

static const struct device *get_icp10125_device(void)
{
	const struct device *dev = DEVICE_DT_GET_ANY(tdk_icp10125);

	if (dev == NULL) {
		printk("\nError: no device found.\n");
		return NULL;
	}

	if (!device_is_ready(dev)) {
		printk("\nError: Device \"%s\" is not ready; "
		       "check the driver initialization logs for errors.\n",
		       dev->name);
		return NULL;
	}

	printk("Found device \"%s\", getting sensor data\n", dev->name);
	return dev;
}

void main(void)
{
	const struct device *dev = get_icp10125_device();
	if (dev == NULL) {
		printk("dev ==null \n");
		return;
	}

	while (1) {
		struct sensor_value temp, press;

		sensor_sample_fetch_chan(dev, SENSOR_CHAN_AMBIENT_TEMP);
		sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
//		printk("temp.val1:%d temp.val2:%06d;\n", temp.val1, temp.val2);
		sensor_sample_fetch_chan(dev, SENSOR_CHAN_PRESS);
		sensor_channel_get(dev, SENSOR_CHAN_PRESS, &press);
//		printk("press.val1: %d press.val2: %06d;\n",press.val1, press.val2);
		printk("temp: %d.%06d; press %d.%06d;\n",temp.val1, temp.val2, press.val1, press.val2);
		k_sleep(K_MSEC(3000));
	}
}
