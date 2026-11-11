# SEN66 I2C driver for Raspberry Pi

A standalone Python driver for the Sensirion SEN66 environmental sensor (PM1–PM10, humidity, temperature, VOC, NOx, CO2). It depends only on `smbus2` and works reliably on the Raspberry Pi 5, including software (bit-banged) I2C buses.

## Why this exists

On a Raspberry Pi 5 with a software I2C bus, the official `sensirion-i2c-sen66` driver kept failing on `read_measured_values` with:

```
Received wrong checksum 0xFF (expected 0xAC)
```

The read was issued before the sensor had staged its response: the official transceiver does not leave a reliable gap between writing the command and reading the data. This driver always uses **two separate I2C transactions with an explicit delay in between** (command → wait → read), the same sequence that works with `i2ctransfer`. Every received word is CRC-checked.

The result types match the official driver, so it can be swapped in without changing the calling code.

## Install

Requires Python 3.10 or newer.

```bash
pip install smbus2
```

Then copy `sen66.py` into your project.

## Quick start

```python
import time
from sen66 import SEN66

with SEN66(bus=3) as sensor:          # bus 3 => /dev/i2c-3, default address 0x6B
    sensor.reset()
    print("serial:", sensor.get_serial_number())
    sensor.start_measurement()
    while True:
        if sensor.get_data_ready():
            m = sensor.read_measured_values()
            print(m.co2, m.temperature, m.humidity)
        time.sleep(1.0)
```

Or run the built-in demo:

```bash
python3 sen66.py --bus 3 --count 10
```

`read_measured_values()` returns a named tuple, so it unpacks exactly like the official driver:

```python
pm1, pm25, pm4, pm10, humidity, temperature, voc_index, nox_index, co2 = sensor.read_measured_values()
```

Values the sensor reports as unknown come back as `None`. Units: PM in µg/m³, humidity in %RH, temperature in °C, VOC and NOx as index values, CO2 in ppm.

## API

| Area | Methods |
| --- | --- |
| Device | `reset()`, `get_serial_number()`, `get_product_name()`, `get_version()`, `read_device_status(clear=False)` |
| Measurement | `start_measurement()`, `stop_measurement()`, `get_data_ready()` |
| Readout | `read_measured_values()`, `read_measured_values_raw()`, `read_number_concentration_values()`, `read_raw_values()` |
| Maintenance | `start_fan_cleaning()`, `activate_sht_heater()` |
| Compensation | `set_temperature_offset()`, `set_temperature_acceleration()`, `set_/get_ambient_pressure()`, `set_/get_sensor_altitude()` |
| CO2 | `set_/get_co2_auto_self_calibration()`, `perform_forced_co2_recalibration()` |
| Gas algorithms | `set_/get_voc_tuning_parameters()`, `set_/get_nox_tuning_parameters()` |

Errors raise `SEN66Error`; a failed checksum raises `SEN66CrcError`, so you can retry the read.

## Using a software I2C bus on the Raspberry Pi

To create `/dev/i2c-3` on two free GPIO pins (here GPIO 23 for SDA and GPIO 24 for SCL), add this to `/boot/firmware/config.txt` and reboot:

```
dtoverlay=i2c-gpio,i2c_gpio_sda=23,i2c_gpio_scl=24,bus=3
```

With the hardware bus (`/dev/i2c-1`), pass `bus=1` instead.

## Background

Written for a distributed IoT system that monitors miniature space greenhouses, where several Raspberry Pi 5 hubs read SEN66 sensors and publish the data over MQTT.
