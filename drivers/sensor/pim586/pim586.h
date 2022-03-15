/*
 * Copyright (c) 2016, 2017 Intel Corporation
 * Copyright (c) 2017 IpTronix S.r.l.
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_PIM586_PIM586_H_
#define ZEPHYR_DRIVERS_SENSOR_PIM586_PIM586_H_

#include <zephyr/types.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/i2c.h>

#define DT_DRV_COMPAT pimoroni_pim586

#define PIM586_BUS_I2C DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)

struct pim586_data {
	/* Compensated values. */
	const struct device *i2c;
	short otp[4];
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
	int 
};

struct pim586_dev_config {
	const char *i2c_master_name;
	uint16_t i2c_addr;
};

#define PIM586_ACC_ODR_100_HZ   0x08
#define PIM586_GYR_ODR_200_HZ   0x09


union pim586_bus {
#if PIM586_BUS_SPI
	struct spi_dt_spec spi;
#endif
#if PIM586_BUS_I2C
	struct i2c_dt_spec i2c;
#endif
};

typedef int (*pim586_bus_check_fn)(const union pim586_bus *bus);
typedef int (*pim586_reg_read_fn)(const union pim586_bus *bus,
				  uint8_t start, uint8_t *buf, int size);
typedef int (*pim586_reg_write_fn)(const union pim586_bus *bus,
				   uint8_t reg, uint8_t val);

struct pim586_bus_io {
	pim586_bus_check_fn check;
	pim586_reg_read_fn read;
	pim586_reg_write_fn write;
};

#if PIM586_BUS_SPI
#define PIM586_SPI_OPERATION (SPI_WORD_SET(8) | SPI_TRANSFER_MSB |	\
			      SPI_MODE_CPOL | SPI_MODE_CPHA)
extern const struct pim586_bus_io pim586_bus_io_spi;
#endif

#if PIM586_BUS_I2C
extern const struct pim586_bus_io pim586_bus_io_i2c;
#endif

#define PIM586_I2C_ADDRESS              DT_INST_REG_ADDR(0)
#define PIM586_REG_SEND                 0xC5


#define PIM586_REG_PRESS_MSB            0xF7
#define PIM586_REG_COMP_START           0x88
#define PIM586_REG_HUM_COMP_PART1       0xA1
#define PIM586_REG_HUM_COMP_PART2       0xE1
#define PIM586_REG_ID                   0xD0
#define PIM586_REG_CONFIG               0xF5
#define PIM586_REG_CTRL_MEAS            0xF4
#define PIM586_REG_CTRL_HUM             0xF2
#define PIM586_REG_STATUS               0xF3
//#define PIM586_REG_RESET                0xE0
#define PIM586_REG_WRITE_HEAD           0xC6
#define PIM586_REG_RESET                0x80
#define PIM586_REG_START_MEAS           0xC7
#define PIM586_REG_START_MEAS_READ      0xF7
#define PIM586_REG_START_MEAS_RETURN    0x9D

#define BMP280_CHIP_ID_SAMPLE_1         0x56
#define BMP280_CHIP_ID_SAMPLE_2         0x57
#define BMP280_CHIP_ID_MP               0x58
#define PIM586_CHIP_ID                  0x60
#define PIM586_MODE_SLEEP               0x00
#define PIM586_MODE_FORCED              0x01
#define PIM586_MODE_NORMAL              0x03
#define PIM586_SPI_3W_DISABLE           0x00
//#define PIM586_CMD_SOFT_RESET           0xB6
#define PIM586_CMD_SOFT_RESET           0x5D
#define PIM586_STATUS_MEASURING         0x08
#define PIM586_STATUS_IM_UPDATE         0x01

#if defined CONFIG_PIM586_MODE_NORMAL
#define PIM586_MODE PIM586_MODE_NORMAL
#elif defined CONFIG_PIM586_MODE_FORCED
#define PIM586_MODE PIM586_MODE_FORCED
#endif

#if defined CONFIG_PIM586_TEMP_OVER_1X
#define PIM586_TEMP_OVER                (1 << 5)
#elif defined CONFIG_PIM586_TEMP_OVER_2X
#define PIM586_TEMP_OVER                (2 << 5)
#elif defined CONFIG_PIM586_TEMP_OVER_4X
#define PIM586_TEMP_OVER                (3 << 5)
#elif defined CONFIG_PIM586_TEMP_OVER_8X
#define PIM586_TEMP_OVER                (4 << 5)
#elif defined CONFIG_PIM586_TEMP_OVER_16X
#define PIM586_TEMP_OVER                (5 << 5)
#endif

#if defined CONFIG_PIM586_PRESS_OVER_1X
#define PIM586_PRESS_OVER               (1 << 2)
#elif defined CONFIG_PIM586_PRESS_OVER_2X
#define PIM586_PRESS_OVER               (2 << 2)
#elif defined CONFIG_PIM586_PRESS_OVER_4X
#define PIM586_PRESS_OVER               (3 << 2)
#elif defined CONFIG_PIM586_PRESS_OVER_8X
#define PIM586_PRESS_OVER               (4 << 2)
#elif defined CONFIG_PIM586_PRESS_OVER_16X
#define PIM586_PRESS_OVER               (5 << 2)
#endif

#if defined CONFIG_PIM586_HUMIDITY_OVER_1X
#define PIM586_HUMIDITY_OVER            1
#elif defined CONFIG_PIM586_HUMIDITY_OVER_2X
#define PIM586_HUMIDITY_OVER            2
#elif defined CONFIG_PIM586_HUMIDITY_OVER_4X
#define PIM586_HUMIDITY_OVER            3
#elif defined CONFIG_PIM586_HUMIDITY_OVER_8X
#define PIM586_HUMIDITY_OVER            4
#elif defined CONFIG_PIM586_HUMIDITY_OVER_16X
#define PIM586_HUMIDITY_OVER            5
#endif

#if defined CONFIG_PIM586_STANDBY_05MS
#define PIM586_STANDBY                  0
#elif defined CONFIG_PIM586_STANDBY_62MS
#define PIM586_STANDBY                  (1 << 5)
#elif defined CONFIG_PIM586_STANDBY_125MS
#define PIM586_STANDBY                  (2 << 5)
#elif defined CONFIG_PIM586_STANDBY_250MS
#define PIM586_STANDBY                  (3 << 5)
#elif defined CONFIG_PIM586_STANDBY_500MS
#define PIM586_STANDBY                  (4 << 5)
#elif defined CONFIG_PIM586_STANDBY_1000MS
#define PIM586_STANDBY                  (5 << 5)
#elif defined CONFIG_PIM586_STANDBY_2000MS
#define PIM586_STANDBY                  (6 << 5)
#elif defined CONFIG_PIM586_STANDBY_4000MS
#define PIM586_STANDBY                  (7 << 5)
#endif

#if defined CONFIG_PIM586_FILTER_OFF
#define PIM586_FILTER                   0
#elif defined CONFIG_PIM586_FILTER_2
#define PIM586_FILTER                   (1 << 2)
#elif defined CONFIG_PIM586_FILTER_4
#define PIM586_FILTER                   (2 << 2)
#elif defined CONFIG_PIM586_FILTER_8
#define PIM586_FILTER                   (3 << 2)
#elif defined CONFIG_PIM586_FILTER_16
#define PIM586_FILTER                   (4 << 2)
#endif

#define PIM586_CTRL_MEAS_VAL            (PIM586_PRESS_OVER | \
					 PIM586_TEMP_OVER |  \
					 PIM586_MODE)
#define PIM586_CONFIG_VAL               (PIM586_STANDBY | \
					 PIM586_FILTER |  \
					 PIM586_SPI_3W_DISABLE)


#define PIM586_CTRL_MEAS_OFF_VAL	(PIM586_PRESS_OVER | \
					 PIM586_TEMP_OVER |  \
					 PIM586_MODE_SLEEP)

#endif /* ZEPHYR_DRIVERS_SENSOR_PIM586_PIM586_H_ */
