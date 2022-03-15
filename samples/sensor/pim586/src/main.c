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
 * "silabs,pim586". (If there are multiple, just pick one.)
 */
static const struct device *get_pim586_device(void)
{
	printk("get-device-1\n ");
	const struct device *dev = DEVICE_DT_GET_ANY(pimoroni_pim586);

	if (dev == NULL) {
		/* No such node, or the node does not have status "okay". */
		printk("\nError: no device found.\n");
		return NULL;
	}

	printk("get-device-2\n ");
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
	printk(" main-1\n ");
	const struct device *dev = get_pim586_device();
	//const struct device *dev = DEVICE_DT_GET(DT_ALIAS(i2c0));
	printk(" main-2 \n ");
	k_sleep(K_MSEC(2000));
	printk(" main-3 \n ");
	if (dev == NULL) {
		printk("dev ==null \n");
		return;
	}
	printk(" main-3 \n ");

	while (1) {
		printk(" while start \n ");
		struct sensor_value temp, press;

		sensor_sample_fetch(dev);
		sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
		sensor_channel_get(dev, SENSOR_CHAN_PRESS, &press);
/*
		sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
		sensor_channel_get(dev, SENSOR_CHAN_PRESS, &press);
		sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY, &humidity);
*/
		printk("restart\n");
		printk("temp: %06d; press: %06d;\n", temp.val1, press.val1);

		k_sleep(K_MSEC(3000));
	}
}
