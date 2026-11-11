#!/usr/bin/env python3
"""
sen66.py - Standalone I2C driver for the Sensirion SEN66 environmental sensor.

Written for Raspberry Pi (incl. Pi 5 with software/bit-banged I2C buses), where
the official sensirion-i2c-sen66 driver can fail on `read_measured_values` with
"Received wrong checksum 0xFF (expected 0xAC)". The cause is that the read is
issued before the sensor has staged its response; the official transceiver path
does not insert a reliable gap between the command write and the data read.

This driver fixes that by always performing two separate I2C transactions with
an explicit delay in between (command -> sleep -> read), exactly like a working
`i2ctransfer` sequence. It depends only on smbus2.

    pip install smbus2

Default address is 0x6B. Default bus is 3 (=> /dev/i2c-3).

Example
-------
    import time

    from sen66 import SEN66

    with SEN66(bus=3) as sensor:
        sensor.reset()
        print("serial:", sensor.get_serial_number())
        sensor.start_measurement()
        while True:
            if sensor.get_data_ready():
                m = sensor.read_measured_values()
                print(m)
            time.sleep(1.0)
"""

import time
from collections import namedtuple

from smbus2 import SMBus, i2c_msg


class SEN66Error(Exception):
    """Base error for SEN66 communication problems."""


class SEN66CrcError(SEN66Error):
    """Raised when a received word fails its CRC check."""


def _crc8(data, poly=0x31, init=0xFF):
    """Sensirion CRC-8: polynomial 0x31, init 0xFF, no final XOR."""
    crc = init
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ poly) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def _u16_to_s16(value):
    """Interpret an unsigned 16-bit value as signed."""
    return value - 0x10000 if value >= 0x8000 else value


# Result types. These are tuples, so positional unpacking works exactly like
# the official driver:
#     pm1, pm25, pm4, pm10, rh, t, voc, nox, co2 = sensor.read_measured_values()
# but you can also use attribute access (m.co2) or m._asdict().
MeasuredValues = namedtuple(
    "MeasuredValues",
    [
        "mass_concentration_pm1p0",
        "mass_concentration_pm2p5",
        "mass_concentration_pm4p0",
        "mass_concentration_pm10p0",
        "humidity",
        "temperature",
        "voc_index",
        "nox_index",
        "co2",
    ],
)
NumberConcentration = namedtuple(
    "NumberConcentration",
    ["pm0p5", "pm1p0", "pm2p5", "pm4p0", "pm10p0"],
)
RawValues = namedtuple(
    "RawValues",
    ["raw_humidity", "raw_temperature", "raw_voc", "raw_nox", "raw_co2"],
)


class SEN66:
    """Driver for a single SEN66 sensor on an I2C bus.

    Parameters
    ----------
    bus : int | str
        I2C bus number (e.g. 3 for /dev/i2c-3) or a device path.
    address : int
        I2C slave address (default 0x6B).
    default_delay : float
        Seconds to wait between a command write and the response read.
        20 ms is safe for all read commands on a bit-banged bus.
    """

    ADDRESS = 0x6B

    # Command IDs (from Sensirion's SEN66 driver). Each is a 16-bit opcode.
    _START_MEASUREMENT = 0x0021
    _STOP_MEASUREMENT = 0x0104
    _GET_DATA_READY = 0x0202
    _READ_MEASURED_VALUES = 0x0300
    _READ_NUMBER_CONCENTRATION = 0x0316
    _READ_RAW_VALUES = 0x0405
    _START_FAN_CLEANING = 0x5607
    _SET_TEMP_OFFSET = 0x60B2
    _VOC_TUNING = 0x60D0  # same opcode for get/set
    _NOX_TUNING = 0x60E1  # same opcode for get/set
    _SET_TEMP_ACCEL = 0x6100
    _VOC_STATE = 0x6181  # same opcode for get/set
    _FORCED_CO2_RECAL = 0x6707
    _CO2_ASC = 0x6711  # same opcode for get/set
    _AMBIENT_PRESSURE = 0x6720  # same opcode for get/set
    _SENSOR_ALTITUDE = 0x6736  # same opcode for get/set
    _ACTIVATE_SHT_HEATER = 0x6765
    _GET_SHT_HEATER_MEAS = 0x6790
    _GET_PRODUCT_NAME = 0xD014
    _GET_SERIAL_NUMBER = 0xD033
    _GET_VERSION = 0xD100
    _READ_DEVICE_STATUS = 0xD206
    _READ_CLEAR_DEVICE_STATUS = 0xD210
    _DEVICE_RESET = 0xD304

    # Device status flag -> bit position (32-bit register, per SEN6x datasheet).
    _STATUS_FLAGS = {
        "fan_error": 4,
        "rht_error": 6,
        "gas_error": 7,
        "co2_2_error": 9,
        "hcho_error": 10,
        "pm_error": 11,
        "co2_1_error": 12,
        "fan_speed_warning": 21,
    }

    def __init__(self, bus=3, address=ADDRESS, default_delay=0.02):
        self.address = address
        self.default_delay = default_delay
        self._own_bus = not isinstance(bus, SMBus)
        self._bus = bus if isinstance(bus, SMBus) else SMBus(bus)

    # ------------------------------------------------------------------ #
    # Context manager
    # ------------------------------------------------------------------ #
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()

    def close(self):
        if self._own_bus and self._bus is not None:
            self._bus.close()
            self._bus = None

    # ------------------------------------------------------------------ #
    # Low-level transport: the core fix lives here
    # ------------------------------------------------------------------ #
    def _frame(self, command, args=()):
        """Build the bytes to write: command + (data word + CRC) per argument."""
        frame = [(command >> 8) & 0xFF, command & 0xFF]
        for arg in args:
            value = arg & 0xFFFF
            hi, lo = (value >> 8) & 0xFF, value & 0xFF
            frame += [hi, lo, _crc8([hi, lo])]
        return frame

    def _write(self, command, args=(), delay=None):
        """Send a command (with optional 16-bit arguments) as one transaction."""
        frame = self._frame(command, args)
        self._bus.i2c_rdwr(i2c_msg.write(self.address, frame))
        time.sleep(self.default_delay if delay is None else delay)

    def _read_words(self, num_words):
        """Read `num_words` 16-bit words (3 bytes each), checking every CRC."""
        msg = i2c_msg.read(self.address, num_words * 3)
        self._bus.i2c_rdwr(msg)
        raw = list(msg)
        words = []
        for i in range(num_words):
            d0, d1, crc = raw[i * 3 : i * 3 + 3]
            if _crc8([d0, d1]) != crc:
                raise SEN66CrcError(
                    f"CRC error on word {i}: data=0x{d0:02X}{d1:02X} "
                    f"recv=0x{crc:02X} calc=0x{_crc8([d0, d1]):02X}"
                )
            words.append((d0 << 8) | d1)
        return words

    def _transceive(self, command, num_words, args=(), delay=None):
        """Write a command then read its response (the safe two-phase pattern)."""
        self._write(command, args, delay)
        return self._read_words(num_words)

    @staticmethod
    def _words_to_ascii(words):
        """Decode a sequence of 16-bit words into a null-terminated ASCII string."""
        chars = bytearray()
        for w in words:
            chars.append((w >> 8) & 0xFF)
            chars.append(w & 0xFF)
        return chars.split(b"\x00", 1)[0].decode("ascii", errors="replace")

    # ------------------------------------------------------------------ #
    # Device info / lifecycle
    # ------------------------------------------------------------------ #
    def reset(self):
        """Soft-reset the device. Wait ~1.2 s afterwards before further use."""
        self._write(self._DEVICE_RESET, delay=1.2)

    def get_serial_number(self):
        """Return the device serial number as a string (up to 32 chars)."""
        return self._words_to_ascii(self._transceive(self._GET_SERIAL_NUMBER, 16))

    def get_product_name(self):
        """Return the product name as a string (e.g. 'SEN66')."""
        return self._words_to_ascii(self._transceive(self._GET_PRODUCT_NAME, 16))

    def get_version(self):
        """Return firmware/hardware/protocol version info.

        word0 = (firmware_major, firmware_minor) is reliable; the remaining
        words are returned raw so you can map them if you need them.
        """
        words = self._transceive(self._GET_VERSION, 3)
        return {
            "firmware_major": (words[0] >> 8) & 0xFF,
            "firmware_minor": words[0] & 0xFF,
            "raw_words": [f"0x{w:04X}" for w in words],
        }

    def read_device_status(self, clear=False):
        """Return the device status as a dict of flag_name -> bool.

        If `clear` is True, the status register is cleared after reading.
        Also includes 'raw' (the 32-bit register value).
        """
        cmd = self._READ_CLEAR_DEVICE_STATUS if clear else self._READ_DEVICE_STATUS
        words = self._transceive(cmd, 2)
        value = (words[0] << 16) | words[1]
        status = {name: bool(value & (1 << bit)) for name, bit in self._STATUS_FLAGS.items()}
        status["raw"] = value
        return status

    # ------------------------------------------------------------------ #
    # Measurement control
    # ------------------------------------------------------------------ #
    def start_measurement(self):
        """Start continuous measurement (1 s interval)."""
        self._write(self._START_MEASUREMENT, delay=0.05)

    def stop_measurement(self):
        """Stop measurement and return to idle. Wait >=1 s before restarting."""
        self._write(self._STOP_MEASUREMENT, delay=1.0)

    def get_data_ready(self):
        """Return True if a new measurement is ready to read."""
        words = self._transceive(self._GET_DATA_READY, 1)
        return bool(words[0] & 0x0001)

    # ------------------------------------------------------------------ #
    # Measurement readout
    # ------------------------------------------------------------------ #
    def read_measured_values_raw(self):
        """Return the 9 raw integer values (no scaling, no sentinel handling)."""
        return self._transceive(self._READ_MEASURED_VALUES, 9)

    def read_measured_values(self):
        """Return scaled measured values as a named tuple.

        Unpacks positionally just like the official driver::

            (pm1, pm25, pm4, pm10, humidity, temperature, voc_index, nox_index, co2) = (
                sensor.read_measured_values()
            )

        and also supports attribute access (``m.co2``) and ``m._asdict()``.
        Unknown values (sensor sentinels 0xFFFF / 0x7FFF) are returned as None.
        Units: PM in ug/m3, humidity in %RH, temperature in degC, VOC/NOx as
        index, CO2 in ppm.
        """
        w = self.read_measured_values_raw()
        return MeasuredValues(
            mass_concentration_pm1p0=None if w[0] == 0xFFFF else w[0] / 10.0,
            mass_concentration_pm2p5=None if w[1] == 0xFFFF else w[1] / 10.0,
            mass_concentration_pm4p0=None if w[2] == 0xFFFF else w[2] / 10.0,
            mass_concentration_pm10p0=None if w[3] == 0xFFFF else w[3] / 10.0,
            humidity=None if w[4] == 0x7FFF else _u16_to_s16(w[4]) / 100.0,
            temperature=None if w[5] == 0x7FFF else _u16_to_s16(w[5]) / 200.0,
            voc_index=None if w[6] == 0x7FFF else _u16_to_s16(w[6]) / 10.0,
            nox_index=None if w[7] == 0x7FFF else _u16_to_s16(w[7]) / 10.0,
            co2=None if w[8] == 0xFFFF else w[8],
        )

    def read_number_concentration_values(self):
        """Return scaled particle number concentrations (particles/cm3).

        Returns a NumberConcentration named tuple (pm0p5, pm1p0, pm2p5,
        pm4p0, pm10p0); unpacks positionally and supports attribute access.
        """
        w = self._transceive(self._READ_NUMBER_CONCENTRATION, 5)
        return NumberConcentration(*[None if v == 0xFFFF else v / 10.0 for v in w])

    def read_raw_values(self):
        """Return uncompensated raw values as a RawValues named tuple
        (raw_humidity, raw_temperature, raw_voc, raw_nox, raw_co2)."""
        w = self._transceive(self._READ_RAW_VALUES, 5)
        return RawValues(
            raw_humidity=None if w[0] == 0x7FFF else _u16_to_s16(w[0]) / 100.0,
            raw_temperature=None if w[1] == 0x7FFF else _u16_to_s16(w[1]) / 200.0,
            raw_voc=None if w[2] == 0xFFFF else w[2],
            raw_nox=None if w[3] == 0xFFFF else w[3],
            raw_co2=None if w[4] == 0xFFFF else w[4],
        )

    # ------------------------------------------------------------------ #
    # Maintenance / calibration
    # ------------------------------------------------------------------ #
    def start_fan_cleaning(self):
        """Run fan cleaning (~10 s, autonomous). Only valid in idle mode."""
        self._write(self._START_FAN_CLEANING, delay=0.02)

    def activate_sht_heater(self):
        """Activate the SHT heater (200 mW for 1 s). Only valid in idle mode."""
        self._write(self._ACTIVATE_SHT_HEATER, delay=0.02)

    def set_temperature_offset(self, offset_c=0.0, slope=0.0, time_constant=0, slot=0):
        """Set a temperature compensation offset slot (0..4)."""
        args = (
            int(round(offset_c * 200)),
            int(round(slope * 10000)),
            int(time_constant),
            int(slot),
        )
        self._write(self._SET_TEMP_OFFSET, args)

    def set_temperature_acceleration(self, k, p, t1, t2):
        """Set RH/T engine acceleration parameters (each scaled x10)."""
        args = (int(round(k * 10)), int(round(p * 10)), int(round(t1 * 10)), int(round(t2 * 10)))
        self._write(self._SET_TEMP_ACCEL, args)

    def set_ambient_pressure(self, hpa):
        """Set ambient pressure (700..1200 hPa) for CO2 compensation."""
        self._write(self._AMBIENT_PRESSURE, (int(hpa),))

    def get_ambient_pressure(self):
        """Get the currently configured ambient pressure in hPa."""
        return self._transceive(self._AMBIENT_PRESSURE, 1)[0]

    def set_sensor_altitude(self, meters):
        """Set sensor altitude (0..3000 m) for CO2 compensation."""
        self._write(self._SENSOR_ALTITUDE, (int(meters),))

    def get_sensor_altitude(self):
        """Get the currently configured sensor altitude in meters."""
        return self._transceive(self._SENSOR_ALTITUDE, 1)[0]

    def set_co2_auto_self_calibration(self, enabled):
        """Enable/disable CO2 automatic self-calibration (ASC)."""
        self._write(self._CO2_ASC, (1 if enabled else 0,))

    def get_co2_auto_self_calibration(self):
        """Return True if CO2 ASC is enabled."""
        return bool(self._transceive(self._CO2_ASC, 1)[0] & 0x0001)

    def perform_forced_co2_recalibration(self, target_ppm):
        """Forced CO2 recalibration. Returns correction in ppm, or None on failure.

        Only valid in idle mode. Takes ~500 ms.
        """
        word = self._transceive(self._FORCED_CO2_RECAL, 1, args=(int(target_ppm),), delay=0.5)[0]
        if word == 0xFFFF:
            return None
        return word - 0x8000

    def get_voc_tuning_parameters(self):
        """Read VOC algorithm tuning parameters as a dict."""
        w = self._transceive(self._VOC_TUNING, 6)
        keys = [
            "index_offset",
            "learning_time_offset_hours",
            "learning_time_gain_hours",
            "gating_max_duration_minutes",
            "std_initial",
            "gain_factor",
        ]
        return {k: _u16_to_s16(v) for k, v in zip(keys, w, strict=True)}

    def set_voc_tuning_parameters(
        self,
        index_offset=100,
        learning_time_offset_hours=12,
        learning_time_gain_hours=12,
        gating_max_duration_minutes=180,
        std_initial=50,
        gain_factor=230,
    ):
        """Set VOC algorithm tuning parameters. Only valid in idle mode."""
        args = (
            index_offset,
            learning_time_offset_hours,
            learning_time_gain_hours,
            gating_max_duration_minutes,
            std_initial,
            gain_factor,
        )
        self._write(self._VOC_TUNING, args)

    def get_nox_tuning_parameters(self):
        """Read NOx algorithm tuning parameters as a dict."""
        w = self._transceive(self._NOX_TUNING, 6)
        keys = [
            "index_offset",
            "learning_time_offset_hours",
            "learning_time_gain_hours",
            "gating_max_duration_minutes",
            "std_initial",
            "gain_factor",
        ]
        return {k: _u16_to_s16(v) for k, v in zip(keys, w, strict=True)}

    def set_nox_tuning_parameters(
        self,
        index_offset=1,
        learning_time_offset_hours=12,
        learning_time_gain_hours=12,
        gating_max_duration_minutes=720,
        std_initial=50,
        gain_factor=230,
    ):
        """Set NOx algorithm tuning parameters. Only valid in idle mode."""
        args = (
            index_offset,
            learning_time_offset_hours,
            learning_time_gain_hours,
            gating_max_duration_minutes,
            std_initial,
            gain_factor,
        )
        self._write(self._NOX_TUNING, args)


def _demo() -> None:
    import argparse

    parser = argparse.ArgumentParser(description="SEN66 quick demo")
    parser.add_argument(
        "--bus", "-b", type=int, default=3, help="I2C bus number (default 3 => /dev/i2c-3)"
    )
    parser.add_argument(
        "--count", "-n", type=int, default=10, help="number of readings (default 10)"
    )
    args = parser.parse_args()

    with SEN66(bus=args.bus) as sensor:  # type: ignore[no-untyped-call]
        sensor.reset()
        print("serial number:", sensor.get_serial_number())
        print("product name :", sensor.get_product_name())
        print("status       :", sensor.read_device_status())
        sensor.start_measurement()
        time.sleep(1.1)
        taken = 0
        while taken < args.count:
            time.sleep(1.0)
            try:
                if not sensor.get_data_ready():
                    continue
                (pm1, pm25, pm4, pm10, humidity, temperature, voc_index, nox_index, co2) = (
                    sensor.read_measured_values()
                )
                print(
                    f"PM2.5={pm25} ug/m3  RH={humidity}%  "
                    f"T={temperature}C  VOC={voc_index}  "
                    f"NOx={nox_index}  CO2={co2} ppm"
                )
                taken += 1
            except SEN66Error as exc:
                print("retry:", exc)
        sensor.stop_measurement()


if __name__ == "__main__":
    _demo()
