# SPDX-FileCopyrightText: 2019 Bryan Siepert for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""
`lsm303dlhc_mag`
====================================================


CircuitPython driver for the LSM303DLHC's magnetometer.

Note that this is specifically intended for the LSM303DLHC, as opposed to the
LSM303DLH proper, which has the magnetic Y and Z orientations swapped.

* Author(s): Dave Astels, Bryan Siepert
* MicroPython port: "robtinkers"

Implementation Notes
--------------------

**Hardware:**

* Adafruit `Triple-axis Accelerometer+Magnetometer (Compass) Board - LSM303
  <https://www.adafruit.com/product/1120>`_ (Product ID: 1120)
* Adafruit `FLORA Accelerometer/Compass Sensor - LSM303 - v1.0
  <https://www.adafruit.com/product/1247>`_ (Product ID: 1247)

**Software and Dependencies:**

* Adafruit CircuitPython firmware:
  https://circuitpython.org/downloads
* Adafruit's Bus Device library:
  https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
"""

from micropython import const
from machine import I2C
import struct

_ADDRESS_MAG = const(0x1E)  # (0x3C >> 1)       // 0011110x
#_ID = const(0xD4)  # (0b11010100)
#MAG_DEVICE_ID = 0b01000000

# Magnetometer registers -- names follow CircuitPython
_REG_MAG_CRA_REG_M = const(0x00)
_REG_MAG_CRB_REG_M = const(0x01)
_REG_MAG_MR_REG_M = const(0x02)
_REG_MAG_OUT_X_H_M = const(0x03)
_REG_MAG_OUT_X_L_M = const(0x04)
_REG_MAG_OUT_Z_H_M = const(0x05)
_REG_MAG_OUT_Z_L_M = const(0x06)
_REG_MAG_OUT_Y_H_M = const(0x07)
_REG_MAG_OUT_Y_L_M = const(0x08)
_REG_MAG_SR_REG_M = const(0x09)
_REG_MAG_IRA_REG_M = const(0x0A)
_REG_MAG_IRB_REG_M = const(0x0B)
_REG_MAG_IRC_REG_M = const(0x0C)
_REG_MAG_TEMP_OUT_H_M = const(0x31)
_REG_MAG_TEMP_OUT_L_M = const(0x32)

# Conversion constants
_GRAVITY_STANDARD = 9.80665  # Earth's gravity in m/s^2
_GAUSS_TO_MICROTESLA = 100.0  # Gauss to micro-Tesla multiplier


# pylint: disable=too-few-public-methods
class Gain:
    """Options for `gain`"""

    GAIN_1_3 = const(1)  # +/- 1.3
    GAIN_1_9 = const(2)  # +/- 1.9
    GAIN_2_5 = const(3)  # +/- 2.5
    GAIN_4_0 = const(4)  # +/- 4.0
    GAIN_4_7 = const(5)  # +/- 4.7
    GAIN_5_6 = const(6)  # +/- 5.6
    GAIN_8_1 = const(7)  # +/- 8.1

class Rate:
    """Options for `rate`"""

    RATE_0_7 = const(0)  # 0.75 Hz
    RATE_1_5 = const(1)  # 1.5 Hz
    RATE_3_0 = const(2)  # 3.0 Hz
    RATE_7_5 = const(3)  # 7.5 Hz
    RATE_15  = const(4)  # 15 Hz
    RATE_30  = const(5)  # 30 Hz
    RATE_75  = const(6)  # 75 Hz
    RATE_220 = const(7)  # 220 Hz


#CircuitPython compatibility constants
MAGGAIN_1_3 = Gain.GAIN_1_3 << 5
MAGGAIN_1_9 = Gain.GAIN_1_9 << 5
MAGGAIN_2_5 = Gain.GAIN_2_5 << 5
MAGGAIN_4_0 = Gain.GAIN_4_0 << 5
MAGGAIN_4_7 = Gain.GAIN_4_7 << 5
MAGGAIN_5_6 = Gain.GAIN_5_6 << 5
MAGGAIN_8_1 = Gain.GAIN_8_1 << 5

MAGRATE_0_7 = Rate.RATE_0_7
MAGRATE_1_5 = Rate.RATE_1_5
MAGRATE_3_0 = Rate.RATE_3_0
MAGRATE_7_5 = Rate.RATE_7_5
MAGRATE_15  = Rate.RATE_15
MAGRATE_30  = Rate.RATE_30
MAGRATE_75  = Rate.RATE_75
MAGRATE_220 = Rate.RATE_220


# pylint: enable=too-few-public-methods


class LSM303DLHC_Mag:
    """Driver for the LSM303DLHC's magnetometer.

    :param ~machine.I2C i2c: The I2C bus the device is connected to.


    **Quickstart: Importing and using the device**

        Here is an example of using the :class:`LSM303DLHC_Mag` class.
        First you will need to import the libraries to use the sensor

        .. code-block:: python

            import machine
            import lsm303dlhc_mag

        Once this is done you can define your `machine.I2C` object and define your sensor object

        .. code-block:: python

            i2c = machine.I2C()  # add parameters
            sensor = lsm303dlhc_mag.LSM303DLHC_Mag(i2c)

        Now you have access to the :attr:`magnetic` attribute

        .. code-block:: python

            mag_x, mag_y, mag_z = sensor.magnetic

    """

    # Class-level buffer for reading and writing data with the sensor.
    # This reduces memory allocations but means the code is not re-entrant or
    # thread safe!
    _BUFFER = bytearray(6)

    def __init__(self, i2c: I2C) -> None:
        self._i2c = i2c
        self._i2c_address = _ADDRESS_MAG
        self._BUFFER1 = memoryview(self._BUFFER)[:1]
        self._BUFFER2 = memoryview(self._BUFFER)[:2]
        self._BUFFER6 = memoryview(self._BUFFER)[:6]

        self._set_register(_REG_MAG_CRA_REG_M, 0)
        self._set_register(_REG_MAG_CRB_REG_M, 0)
        self._set_register(_REG_MAG_MR_REG_M, 0)

        self.gain = Gain.GAIN_1_3
        self.rate = Rate.RATE_0_7

    @property
    def _raw_magnetic(self) -> tuple: # Tuple[int, int, int]:
        """The raw magnetometer sensor values.
        A 3-tuple of X, Y, Z axis values that are 16-bit signed integers.
        """
        self._read_bytes(_REG_MAG_OUT_X_H_M, self._BUFFER6)
        return struct.unpack_from(">hhh", self._BUFFER6)

    @property
    def magnetic(self) -> Tuple[float, float, float]:
        """The processed magnetometer sensor values.
        A 3-tuple of X, Y, Z axis values in microteslas that are signed floats.
        """
        self._read_bytes(_REG_MAG_OUT_X_H_M, self._BUFFER6)
        raw_mag_data = struct.unpack_from(">hhh", self._BUFFER6)
        raw_x, raw_y, raw_z = raw_mag_data
        return (
            raw_x / self._lsm303mag_gauss_lsb_xy * _GAUSS_TO_MICROTESLA,
            raw_y / self._lsm303mag_gauss_lsb_xy * _GAUSS_TO_MICROTESLA,
            raw_z / self._lsm303mag_gauss_lsb_z * _GAUSS_TO_MICROTESLA,
        )

    @property
    def gain(self) -> int:
        """The magnetometer's gain."""
        return self._cached_gain

    @gain.setter
    def gain(self, value: int) -> None:
        if value not in (
            Gain.GAIN_1_3,
            Gain.GAIN_1_9,
            Gain.GAIN_2_5,
            Gain.GAIN_4_0,
            Gain.GAIN_4_7,
            Gain.GAIN_5_6,
            Gain.GAIN_8_1,
        ):
            raise AttributeError("gain must be a `Gain`")

        self._set_register(_REG_MAG_CRB_REG_M, value << 5, mask=0b_11100000)
        self._cached_gain = value

        if self._cached_gain == Gain.GAIN_1_3:
            self._lsm303mag_gauss_lsb_xy = 1100.0
            self._lsm303mag_gauss_lsb_z = 980.0
        elif self._cached_gain == Gain.GAIN_1_9:
            self._lsm303mag_gauss_lsb_xy = 855.0
            self._lsm303mag_gauss_lsb_z = 760.0
        elif self._cached_gain == Gain.GAIN_2_5:
            self._lsm303mag_gauss_lsb_xy = 670.0
            self._lsm303mag_gauss_lsb_z = 600.0
        elif self._cached_gain == Gain.GAIN_4_0:
            self._lsm303mag_gauss_lsb_xy = 450.0
            self._lsm303mag_gauss_lsb_z = 400.0
        elif self._cached_gain == Gain.GAIN_4_7:
            self._lsm303mag_gauss_lsb_xy = 400.0
            self._lsm303mag_gauss_lsb_z = 355.0
        elif self._cached_gain == Gain.GAIN_5_6:
            self._lsm303mag_gauss_lsb_xy = 330.0
            self._lsm303mag_gauss_lsb_z = 295.0
        elif self._cached_gain == Gain.GAIN_8_1:
            self._lsm303mag_gauss_lsb_xy = 230.0
            self._lsm303mag_gauss_lsb_z = 205.0

    @property
    def rate(self) -> int:
        """The magnetometer update rate."""
        return self._cached_rate

    @rate.setter
    def rate(self, value: int) -> None:
        if value not in (
            Rate.RATE_0_7,
            Rate.RATE_1_5,
            Rate.RATE_3_0,
            Rate.RATE_7_5,
            Rate.RATE_15,
            Rate.RATE_30,
            Rate.RATE_75,
            Rate.RATE_220,
        ):
            raise AttributeError("rate must be a `Rate`")

        self._set_register(_REG_MAG_CRA_REG_M, value << 2, mask=0b_00011100)
        self._cached_rate = value

    @property
    def mag_gain(self) -> int:
        """CircuitPython compatibility."""
        return self.gain << 5

    @mag_gain.setter
    def mag_gain(self, value: int) -> None:
        self.gain = value >> 5

    @property
    def mag_rate(self) -> int:
        """CircuitPython compatibility."""
        return self.rate

    @mag_rate.setter
    def mag_rate(self, value: int) -> None:
        self.rate = value

    def _get_register(self, reg: int) -> int:
        self._BUFFER1[0] = reg
        self._i2c.writeto(self._i2c_address, self._BUFFER1)
        self._i2c.readfrom_into(self._i2c_address, self._BUFFER1)
        return self._BUFFER1[0]

    def _set_register(self, reg: int, val: int, *, mask=None) -> None:
        if mask is not None:
            val = (val & mask) | (self._get_register(reg) & ~mask)
        self._BUFFER2[0] = reg
        self._BUFFER2[1] = val
        self._i2c.writeto(self._i2c_address, self._BUFFER2)

    def _read_bytes(self, reg: int, buf: bytearray) -> None:
        self._BUFFER1[0] = reg
        self._i2c.writeto(self._i2c_address, self._BUFFER1)
        self._i2c.readfrom_into(self._i2c_address, buf)