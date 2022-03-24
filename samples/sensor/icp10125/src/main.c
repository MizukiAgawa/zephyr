/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/sensor.h>

/*
 * Get a device structure from a devicetree node with compatible
 * "silabs,icp10125". (If there are multiple, just pick one.)
 */
static const struct device *get_icp10125_device(void)
{
	const struct device *dev = DEVICE_DT_GET_ANY(tdk_icp10125);

	if (dev == NULL) {
		/* No such node, or the node does not have status "okay". */
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
	k_sleep(K_MSEC(2000));
	const struct device *dev = get_icp10125_device();
	k_sleep(K_MSEC(2000));
	if (dev == NULL) {
		printk("dev ==null \n");
		return;
	}

	while (1) {
		struct sensor_value temp, press;

		sensor_sample_fetch_chan(dev, SENSOR_CHAN_AMBIENT_TEMP);
		sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
		printk("temp.val1:%d temp.val2:%06d;\n", temp.val1, temp.val2);
		sensor_sample_fetch_chan(dev, SENSOR_CHAN_PRESS);
		sensor_channel_get(dev, SENSOR_CHAN_PRESS, &press);
		printk("press.val1: %d press.val2: %06d;\n",press.val1, press.val2);
		k_sleep(K_MSEC(3000));
	}
}
