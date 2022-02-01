/*
 * Copyright (c) 2016, 2017 Intel Corporation
 * Copyright (c) 2017 IpTronix S.r.l.
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_SI1145_SI1145_H_
#define ZEPHYR_DRIVERS_SENSOR_SI1145_SI1145_H_

#include <zephyr/types.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/i2c.h>

#define DT_DRV_COMPAT silabs_si1145

#define SI1145_BUS_I2C DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)

union si1145_bus {
#if SI1145_BUS_SPI
	struct spi_dt_spec spi;
#endif
#if SI1145_BUS_I2C
	struct i2c_dt_spec i2c;
#endif
};

typedef int (*si1145_bus_check_fn)(const union si1145_bus *bus);
typedef int (*si1145_reg_read_fn)(const union si1145_bus *bus,
				  uint8_t start, uint8_t *buf, int size);
typedef int (*si1145_reg_write_fn)(const union si1145_bus *bus,
				   uint8_t reg, uint8_t val);

struct si1145_bus_io {
	si1145_bus_check_fn check;
	si1145_reg_read_fn read;
	si1145_reg_write_fn write;
};

#if SI1145_BUS_SPI
#define SI1145_SPI_OPERATION (SPI_WORD_SET(8) | SPI_TRANSFER_MSB |	\
			      SPI_MODE_CPOL | SPI_MODE_CPHA)
extern const struct si1145_bus_io si1145_bus_io_spi;
#endif

#if SI1145_BUS_I2C
extern const struct si1145_bus_io si1145_bus_io_i2c;
#endif

#define SI1145_REG_PRESS_MSB            0xF7
#define SI1145_REG_COMP_START           0x88
#define SI1145_REG_HUM_COMP_PART1       0xA1
#define SI1145_REG_HUM_COMP_PART2       0xE1
#define SI1145_REG_ID                   0xD0
#define SI1145_REG_CONFIG               0xF5
#define SI1145_REG_CTRL_MEAS            0xF4
#define SI1145_REG_CTRL_HUM             0xF2
#define SI1145_REG_STATUS               0xF3
#define SI1145_REG_RESET                0xE0

#define BMP280_CHIP_ID_SAMPLE_1         0x56
#define BMP280_CHIP_ID_SAMPLE_2         0x57
#define BMP280_CHIP_ID_MP               0x58
#define SI1145_CHIP_ID                  0x60
#define SI1145_MODE_SLEEP               0x00
#define SI1145_MODE_FORCED              0x01
#define SI1145_MODE_NORMAL              0x03
#define SI1145_SPI_3W_DISABLE           0x00
#define SI1145_CMD_SOFT_RESET           0xB6
#define SI1145_STATUS_MEASURING         0x08
#define SI1145_STATUS_IM_UPDATE         0x01

#if defined CONFIG_SI1145_MODE_NORMAL
#define SI1145_MODE SI1145_MODE_NORMAL
#elif defined CONFIG_SI1145_MODE_FORCED
#define SI1145_MODE SI1145_MODE_FORCED
#endif

#if defined CONFIG_SI1145_TEMP_OVER_1X
#define SI1145_TEMP_OVER                (1 << 5)
#elif defined CONFIG_SI1145_TEMP_OVER_2X
#define SI1145_TEMP_OVER                (2 << 5)
#elif defined CONFIG_SI1145_TEMP_OVER_4X
#define SI1145_TEMP_OVER                (3 << 5)
#elif defined CONFIG_SI1145_TEMP_OVER_8X
#define SI1145_TEMP_OVER                (4 << 5)
#elif defined CONFIG_SI1145_TEMP_OVER_16X
#define SI1145_TEMP_OVER                (5 << 5)
#endif

#if defined CONFIG_SI1145_PRESS_OVER_1X
#define SI1145_PRESS_OVER               (1 << 2)
#elif defined CONFIG_SI1145_PRESS_OVER_2X
#define SI1145_PRESS_OVER               (2 << 2)
#elif defined CONFIG_SI1145_PRESS_OVER_4X
#define SI1145_PRESS_OVER               (3 << 2)
#elif defined CONFIG_SI1145_PRESS_OVER_8X
#define SI1145_PRESS_OVER               (4 << 2)
#elif defined CONFIG_SI1145_PRESS_OVER_16X
#define SI1145_PRESS_OVER               (5 << 2)
#endif

#if defined CONFIG_SI1145_HUMIDITY_OVER_1X
#define SI1145_HUMIDITY_OVER            1
#elif defined CONFIG_SI1145_HUMIDITY_OVER_2X
#define SI1145_HUMIDITY_OVER            2
#elif defined CONFIG_SI1145_HUMIDITY_OVER_4X
#define SI1145_HUMIDITY_OVER            3
#elif defined CONFIG_SI1145_HUMIDITY_OVER_8X
#define SI1145_HUMIDITY_OVER            4
#elif defined CONFIG_SI1145_HUMIDITY_OVER_16X
#define SI1145_HUMIDITY_OVER            5
#endif

#if defined CONFIG_SI1145_STANDBY_05MS
#define SI1145_STANDBY                  0
#elif defined CONFIG_SI1145_STANDBY_62MS
#define SI1145_STANDBY                  (1 << 5)
#elif defined CONFIG_SI1145_STANDBY_125MS
#define SI1145_STANDBY                  (2 << 5)
#elif defined CONFIG_SI1145_STANDBY_250MS
#define SI1145_STANDBY                  (3 << 5)
#elif defined CONFIG_SI1145_STANDBY_500MS
#define SI1145_STANDBY                  (4 << 5)
#elif defined CONFIG_SI1145_STANDBY_1000MS
#define SI1145_STANDBY                  (5 << 5)
#elif defined CONFIG_SI1145_STANDBY_2000MS
#define SI1145_STANDBY                  (6 << 5)
#elif defined CONFIG_SI1145_STANDBY_4000MS
#define SI1145_STANDBY                  (7 << 5)
#endif

#if defined CONFIG_SI1145_FILTER_OFF
#define SI1145_FILTER                   0
#elif defined CONFIG_SI1145_FILTER_2
#define SI1145_FILTER                   (1 << 2)
#elif defined CONFIG_SI1145_FILTER_4
#define SI1145_FILTER                   (2 << 2)
#elif defined CONFIG_SI1145_FILTER_8
#define SI1145_FILTER                   (3 << 2)
#elif defined CONFIG_SI1145_FILTER_16
#define SI1145_FILTER                   (4 << 2)
#endif

#define SI1145_CTRL_MEAS_VAL            (SI1145_PRESS_OVER | \
					 SI1145_TEMP_OVER |  \
					 SI1145_MODE)
#define SI1145_CONFIG_VAL               (SI1145_STANDBY | \
					 SI1145_FILTER |  \
					 SI1145_SPI_3W_DISABLE)


#define SI1145_CTRL_MEAS_OFF_VAL	(SI1145_PRESS_OVER | \
					 SI1145_TEMP_OVER |  \
					 SI1145_MODE_SLEEP)

#endif /* ZEPHYR_DRIVERS_SENSOR_SI1145_SI1145_H_ */
