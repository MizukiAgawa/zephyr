/*
 * Copyright (c) 2016, 2017 Intel Corporation
 * Copyright (c) 2017 IpTronix S.r.l.
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_ICP10125_ICP10125_H_
#define ZEPHYR_DRIVERS_SENSOR_ICP10125_ICP10125_H_

#include <zephyr/types.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/i2c.h>

#define DT_DRV_COMPAT tdk_icp10125

#define ICP10125_BUS_I2C DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)

struct icp10125_data {
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
	int T_LSB;
	int p_LSB;
};

struct icp10125_dev_config {
	const char *i2c_master_name;
	uint16_t i2c_addr;
};

#define ICP10125_ACC_ODR_100_HZ   0x08
#define ICP10125_GYR_ODR_200_HZ   0x09


union icp10125_bus {
#if ICP10125_BUS_SPI
	struct spi_dt_spec spi;
#endif
#if ICP10125_BUS_I2C
	struct i2c_dt_spec i2c;
#endif
};

typedef int (*icp10125_bus_check_fn)(const union icp10125_bus *bus);
typedef int (*icp10125_reg_read_fn)(const union icp10125_bus *bus,
				  uint8_t start, uint8_t *buf, int size);
typedef int (*icp10125_reg_write_fn)(const union icp10125_bus *bus,
				   uint8_t reg, uint8_t val);

struct icp10125_bus_io {
	icp10125_bus_check_fn check;
	icp10125_reg_read_fn read;
	icp10125_reg_write_fn write;
};

#if ICP10125_BUS_SPI
#define ICP10125_SPI_OPERATION (SPI_WORD_SET(8) | SPI_TRANSFER_MSB |	\
			      SPI_MODE_CPOL | SPI_MODE_CPHA)
extern const struct icp10125_bus_io icp10125_bus_io_spi;
#endif

#if ICP10125_BUS_I2C
extern const struct icp10125_bus_io icp10125_bus_io_i2c;
#endif

#define ICP10125_I2C_ADDRESS              DT_INST_REG_ADDR(0)
#define ICP10125_REG_SEND                 0xC5


#define ICP10125_REG_PRESS_MSB            0xF7
#define ICP10125_REG_COMP_START           0x88
#define ICP10125_REG_HUM_COMP_PART1       0xA1
#define ICP10125_REG_HUM_COMP_PART2       0xE1
#define ICP10125_REG_ID                   0xD0
#define ICP10125_REG_CONFIG               0xF5
#define ICP10125_REG_CTRL_MEAS            0xF4
#define ICP10125_REG_CTRL_HUM             0xF2
#define ICP10125_REG_STATUS               0xF3
//#define ICP10125_REG_RESET                0xE0
#define ICP10125_REG_WRITE_HEAD           0xC6
#define ICP10125_REG_RESET                0x80
#define ICP10125_REG_START_MEAS           0xC7
#define ICP10125_REG_START_MEAS_READ      0xF7
#define ICP10125_REG_START_MEAS_RETURN    0x9D

#define BMP280_CHIP_ID_SAMPLE_1         0x56
#define BMP280_CHIP_ID_SAMPLE_2         0x57
#define BMP280_CHIP_ID_MP               0x58
#define ICP10125_CHIP_ID                  0x60
#define ICP10125_MODE_SLEEP               0x00
#define ICP10125_MODE_FORCED              0x01
#define ICP10125_MODE_NORMAL              0x03
#define ICP10125_SPI_3W_DISABLE           0x00
//#define ICP10125_CMD_SOFT_RESET           0xB6
#define ICP10125_CMD_SOFT_RESET           0x5D
#define ICP10125_STATUS_MEASURING         0x08
#define ICP10125_STATUS_IM_UPDATE         0x01

#if defined CONFIG_ICP10125_MODE_NORMAL
#define ICP10125_MODE ICP10125_MODE_NORMAL
#elif defined CONFIG_ICP10125_MODE_FORCED
#define ICP10125_MODE ICP10125_MODE_FORCED
#endif

#if defined CONFIG_ICP10125_TEMP_OVER_1X
#define ICP10125_TEMP_OVER                (1 << 5)
#elif defined CONFIG_ICP10125_TEMP_OVER_2X
#define ICP10125_TEMP_OVER                (2 << 5)
#elif defined CONFIG_ICP10125_TEMP_OVER_4X
#define ICP10125_TEMP_OVER                (3 << 5)
#elif defined CONFIG_ICP10125_TEMP_OVER_8X
#define ICP10125_TEMP_OVER                (4 << 5)
#elif defined CONFIG_ICP10125_TEMP_OVER_16X
#define ICP10125_TEMP_OVER                (5 << 5)
#endif

#if defined CONFIG_ICP10125_PRESS_OVER_1X
#define ICP10125_PRESS_OVER               (1 << 2)
#elif defined CONFIG_ICP10125_PRESS_OVER_2X
#define ICP10125_PRESS_OVER               (2 << 2)
#elif defined CONFIG_ICP10125_PRESS_OVER_4X
#define ICP10125_PRESS_OVER               (3 << 2)
#elif defined CONFIG_ICP10125_PRESS_OVER_8X
#define ICP10125_PRESS_OVER               (4 << 2)
#elif defined CONFIG_ICP10125_PRESS_OVER_16X
#define ICP10125_PRESS_OVER               (5 << 2)
#endif

#if defined CONFIG_ICP10125_HUMIDITY_OVER_1X
#define ICP10125_HUMIDITY_OVER            1
#elif defined CONFIG_ICP10125_HUMIDITY_OVER_2X
#define ICP10125_HUMIDITY_OVER            2
#elif defined CONFIG_ICP10125_HUMIDITY_OVER_4X
#define ICP10125_HUMIDITY_OVER            3
#elif defined CONFIG_ICP10125_HUMIDITY_OVER_8X
#define ICP10125_HUMIDITY_OVER            4
#elif defined CONFIG_ICP10125_HUMIDITY_OVER_16X
#define ICP10125_HUMIDITY_OVER            5
#endif

#if defined CONFIG_ICP10125_STANDBY_05MS
#define ICP10125_STANDBY                  0
#elif defined CONFIG_ICP10125_STANDBY_62MS
#define ICP10125_STANDBY                  (1 << 5)
#elif defined CONFIG_ICP10125_STANDBY_125MS
#define ICP10125_STANDBY                  (2 << 5)
#elif defined CONFIG_ICP10125_STANDBY_250MS
#define ICP10125_STANDBY                  (3 << 5)
#elif defined CONFIG_ICP10125_STANDBY_500MS
#define ICP10125_STANDBY                  (4 << 5)
#elif defined CONFIG_ICP10125_STANDBY_1000MS
#define ICP10125_STANDBY                  (5 << 5)
#elif defined CONFIG_ICP10125_STANDBY_2000MS
#define ICP10125_STANDBY                  (6 << 5)
#elif defined CONFIG_ICP10125_STANDBY_4000MS
#define ICP10125_STANDBY                  (7 << 5)
#endif

#if defined CONFIG_ICP10125_FILTER_OFF
#define ICP10125_FILTER                   0
#elif defined CONFIG_ICP10125_FILTER_2
#define ICP10125_FILTER                   (1 << 2)
#elif defined CONFIG_ICP10125_FILTER_4
#define ICP10125_FILTER                   (2 << 2)
#elif defined CONFIG_ICP10125_FILTER_8
#define ICP10125_FILTER                   (3 << 2)
#elif defined CONFIG_ICP10125_FILTER_16
#define ICP10125_FILTER                   (4 << 2)
#endif

#define ICP10125_CTRL_MEAS_VAL            (ICP10125_PRESS_OVER | \
					 ICP10125_TEMP_OVER |  \
					 ICP10125_MODE)
#define ICP10125_CONFIG_VAL               (ICP10125_STANDBY | \
					 ICP10125_FILTER |  \
					 ICP10125_SPI_3W_DISABLE)


#define ICP10125_CTRL_MEAS_OFF_VAL	(ICP10125_PRESS_OVER | \
					 ICP10125_TEMP_OVER |  \
					 ICP10125_MODE_SLEEP)

#endif /* ZEPHYR_DRIVERS_SENSOR_ICP10125_ICP10125_H_ */
