.. _si1145:

SI1145 Humidity and Pressure Sensor
###################################

Overview
********

This sample shows how to use the Zephyr :ref:`sensor_api` API driver for the
`Bosch SI1145`_ environmental sensor.

.. _Bosch SI1145:
   https://www.bosch-sensortec.com/products/environmental-sensors/humidity-sensors-si1145/`

The sample periodically reads temperature, pressure and humidity data from the
first available SI1145 device discovered in the system. The sample checks the
sensor in polling mode (without interrupt trigger).

Building and Running
********************

The sample can be configured to support SI1145 sensors connected via either I2C
or SPI. Configuration is done via :ref:`devicetree <dt-guide>`. The devicetree
must have an enabled node with ``compatible = "bosch,si1145";``. See
:dtcompatible:`bosch,si1145` for the devicetree binding and see below for
examples and common configurations.

If the sensor is not built into your board, start by wiring the sensor pins
according to the connection diagram given in the `SI1145 datasheet`_ at
page 38.

.. _SI1145 datasheet:
   https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-si1145-ds002.pdf

Boards with a built-in SI1145
=============================

Your board may have a SI1145 node configured in its devicetree by default. Make
sure this node has ``status = "okay";``, then build and run with:

.. zephyr-app-commands::
   :zephyr-app: samples/sensor/si1145
   :goals: build flash
   :board: adafruit_feather_m0_basic_proto

SI1145 via Arduino SPI pins
===========================

If you wired the sensor to a SPI peripheral on an Arduino header, build and
flash with:

.. zephyr-app-commands::
   :zephyr-app: samples/sensor/si1145
   :goals: build flash
   :gen-args: -DDTC_OVERLAY_FILE=arduino_spi.overlay

The devicetree overlay :zephyr_file:`samples/sensor/si1145/arduino_spi.overlay`
works on any board with a properly configured Arduino pin-compatible SPI
peripheral.

SI1145 via Arduino I2C pins
===========================

If you wired the sensor to an I2C peripheral on an Arduino header, build and
flash with:

.. zephyr-app-commands::
   :zephyr-app: samples/sensor/si1145
   :goals: build flash
   :gen-args: -DDTC_OVERLAY_FILE=arduino_i2c.overlay

The devicetree overlay :zephyr_file:`samples/sensor/si1145/arduino_i2c.overlay`
works on any board with a properly configured Arduino pin-compatible I2C
peripheral.

Board-specific overlays
=======================

If your board's devicetree does not have a SI1145 node already, you can create
a board-specific devicetree overlay adding one in the :file:`boards` directory.
See existing overlays for examples.

The build system uses these overlays by default when targeting those boards, so
no ``DTC_OVERLAY_FILE`` setting is needed when building and running.

For example, to build for the :ref:`adafruit_feather_m0_basic_proto` using the
:zephyr_file:`samples/sensor/si1145/boards/adafruit_feather_m0_basic_proto.overlay`
overlay provided with this sample:

.. zephyr-app-commands::
   :zephyr-app: samples/sensor/si1145
   :goals: build flash
   :board: adafruit_feather_m0_basic_proto

Sample Output
=============

The sample prints output to the serial console. SI1145 device driver messages
are also logged. Refer to your board's documentation for information on
connecting to its serial console.

Here is example output for the default application settings, assuming that only
one SI1145 sensor is connected to the standard Arduino I2C pins:

.. code-block:: none

   [00:00:00.379,760] <dbg> SI1145.si1145_init: initializing "SI1145_SPI" on bus "SPI_3"
   [00:00:00.379,821] <dbg> SI1145.si1145_init: bad chip id 0xff
   [00:00:00.379,821] <dbg> SI1145.si1145_init: initializing "SI1145_I2C" on bus "I2C_0"
   [00:00:00.380,340] <dbg> SI1145.si1145_init: ID OK
   [00:00:00.385,559] <dbg> SI1145.si1145_init: SI1145_I2C OK
   *** Booting Zephyr OS build zephyr-v2.4.0-2940-gbb732ada394f  ***
   Found device SI1145_I2C, getting sensor data
   temp: 20.260000; press: 99.789019; humidity: 46.458984
   temp: 20.260000; press: 99.789480; humidity: 46.424804
   temp: 20.250000; press: 99.789246; humidity: 46.423828

Here is example output for the default application settings, assuming that two
different SI1145 sensors are connected to the standard Arduino I2C and SPI pins:

.. code-block:: none

   [00:00:00.377,777] <dbg> SI1145.si1145_init: initializing "SI1145_SPI" on bus "SPI_3"
   [00:00:00.377,838] <dbg> SI1145.si1145_init: ID OK
   [00:00:00.379,608] <dbg> SI1145.si1145_init: SI1145_SPI OK
   [00:00:00.379,638] <dbg> SI1145.si1145_init: initializing "SI1145_I2C" on bus "I2C_0"
   [00:00:00.380,126] <dbg> SI1145.si1145_init: ID OK
   [00:00:00.385,345] <dbg> SI1145.si1145_init: SI1145_I2C OK
   *** Booting Zephyr OS build zephyr-v2.4.0-2940-gbb732ada394f  ***
   Found device SI1145_I2C, getting sensor data
   temp: 20.150000; press: 99.857675; humidity: 46.447265
   temp: 20.150000; press: 99.859121; humidity: 46.458984
   temp: 20.150000; press: 99.859234; humidity: 46.469726

That the driver logs include a line saying ``SI1145_I2C OK`` in both cases, but
``SI1145_SPI OK`` is missing when that device is not connected.
