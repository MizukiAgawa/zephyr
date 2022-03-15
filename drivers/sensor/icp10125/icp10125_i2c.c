/*
 * Copyright (c) 2016, 2017 Intel Corporation
 * Copyright (c) 2017 IpTronix S.r.l.
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Bus-specific functionality for PIM586s accessed via I2C.
 */

#include "pim586.h"

#if PIM586_BUS_I2C
static int pim586_bus_check_i2c(const union pim586_bus *bus)
{
	return device_is_ready(bus->i2c.bus) ? 0 : -ENODEV;
}

static int pim586_reg_read_i2c(const union pim586_bus *bus,
			       uint8_t start, uint8_t *buf, int size)
{
	return i2c_burst_read_dt(&bus->i2c, start, buf, size);
}

static int pim586_reg_write_i2c(const union pim586_bus *bus,
				uint8_t reg, uint8_t val)
{
	return i2c_reg_write_byte_dt(&bus->i2c, reg, val);
}

const struct pim586_bus_io pim586_bus_io_i2c = {
	.check = pim586_bus_check_i2c,
	.read = pim586_reg_read_i2c,
	.write = pim586_reg_write_i2c,
};
#endif /* PIM586_BUS_I2C */
