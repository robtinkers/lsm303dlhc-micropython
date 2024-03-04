# SPDX-FileCopyrightText: 2019 Bryan Siepert for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""
`lsm303dlhc_accel`
====================================================


CircuitPython driver for the accelerometer in LSM303DLHC sensors.

* Author(s): Dave Astels, Bryan Siepert
* MicroPython port: "robtinkers"

Implementation Notes
--------------------

**Hardware:**

* Adafruit `Triple-axis Accelerometer+Magnetometer (Compass) Board - LSM303DLHC
  <https://www.adafruit.com/product/1120>`_ (Product ID: 1120)
* Adafruit `FLORA Accelerometer/Compass Sensor - LSM303DLHC - v1.0
  <https://www.adafruit.com/product/1247>`_ (Product ID: 1247)

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://circuitpython.org/downloads
* Adafruit's Bus Device library:
  https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
"""

from micropython import const
from machine import I2C
import struct

_ADDRESS_ACCEL = const(0x19)  # (0x32 >> 1)       // 0011001x
#_ID = const(0xD4)  # (0b11010100)

# Accelerometer registers -- names follow CircuitPython
_REG_ACCEL_CTRL_REG1_A = const(0x20)
_REG_ACCEL_CTRL_REG2_A = const(0x21)
_REG_ACCEL_CTRL_REG3_A = const(0x22)
_REG_ACCEL_CTRL_REG4_A = const(0x23)
_REG_ACCEL_CTRL_REG5_A = const(0x24)
_REG_ACCEL_CTRL_REG6_A = const(0x25)
_REG_ACCEL_REFERENCE_A = const(0x26)
_REG_ACCEL_STATUS_REG_A = const(0x27)
_REG_ACCEL_OUT_X_L_A = const(0x28)
_REG_ACCEL_OUT_X_H_A = const(0x29)
_REG_ACCEL_OUT_Y_L_A = const(0x2A)
_REG_ACCEL_OUT_Y_H_A = const(0x2B)
_REG_ACCEL_OUT_Z_L_A = const(0x2C)
_REG_ACCEL_OUT_Z_H_A = const(0x2D)
_REG_ACCEL_FIFO_CTRL_REG_A = const(0x2E)
_REG_ACCEL_FIFO_SRC_REG_A = const(0x2F)
_REG_ACCEL_INT1_CFG_A = const(0x30)
_REG_ACCEL_INT1_SOURCE_A = const(0x31)
_REG_ACCEL_INT1_THS_A = const(0x32)
_REG_ACCEL_INT1_DURATION_A = const(0x33)
_REG_ACCEL_INT2_CFG_A = const(0x34)
_REG_ACCEL_INT2_SOURCE_A = const(0x35)
_REG_ACCEL_INT2_THS_A = const(0x36)
_REG_ACCEL_INT2_DURATION_A = const(0x37)
_REG_ACCEL_CLICK_CFG_A = const(0x38)
_REG_ACCEL_CLICK_SRC_A = const(0x39)
_REG_ACCEL_CLICK_THS_A = const(0x3A)
_REG_ACCEL_TIME_LIMIT_A = const(0x3B)
_REG_ACCEL_TIME_LATENCY_A = const(0x3C)
_REG_ACCEL_TIME_WINDOW_A = const(0x3D)
# note:: Tap related registers are called ``CLICK_`` in the datasheet

# Conversion constants
_LSM303ACCEL_MG_LSB = 16704.0  # magic!
_GRAVITY_STANDARD = 9.80665  # Earth's gravity in m/s^2
_SMOLLER_GRAVITY = 0.00980665


# pylint: disable=too-few-public-methods
class Rate:
    """Options for `data_rate`"""

    RATE_SHUTDOWN = const(0)
    RATE_1_HZ = const(1)       # normal / low-power mode
    RATE_10_HZ = const(2)      # normal / low-power mode
    RATE_25_HZ = const(3)      # normal / low-power mode
    RATE_50_HZ = const(4)      # normal / low-power mode
    RATE_100_HZ = const(5)     # normal / low-power mode
    RATE_200_HZ = const(6)     # normal / low-power mode
    RATE_400_HZ = const(7)     # normal / low-power mode
    RATE_1620_HZ = const(8)    # low-power mode
    RATE_1344_HZ = const(9)    # normal mode
    RATE_5376_HZ = const(10)   # low-power mode, special handling in .rate()


class Mode:
    """Options for `mode`"""

    MODE_NORMAL = const(0)
    MODE_HIGH_RESOLUTION = const(1)
    MODE_LOW_POWER = const(2)


class Range:
    """Options for `range`"""

    RANGE_2G = const(0)
    RANGE_4G = const(1)
    RANGE_8G = const(2)
    RANGE_16G = const(3)


# pylint: enable=too-few-public-methods


class LSM303DLHC_Accel:  # pylint:disable=too-many-instance-attributes
    """Driver for the LSM303DLHC's accelerometer.

    :param ~machine.I2C i2c: The I2C bus the device is connected to.


    **Quickstart: Importing and using the device**

        Here is an example of using the :class:`LSM303DLHC_Accel` class.
        First you will need to import the libraries to use the sensor

        .. code-block:: python

            import machine
            import lsm303dlhc_accel

        Once this is done you can define your `machine.I2C` object and define your sensor object

        .. code-block:: python

            i2c = machine.I2C()  # add parameters
            sensor = lsm303dlhc_accel.LSM303DLHC_Accel(i2c)

        Now you have access to the :attr:`acceleration` attribute

        .. code-block:: python

            acc_x, acc_y, acc_z = sensor.acceleration


    """

    # Class-level buffer for reading and writing data with the sensor.
    # This reduces memory allocations but means the code is not re-entrant or
    # thread safe!

    _BUFFER = bytearray(6)

    def __init__(self, i2c: I2C) -> None:
        self._i2c = i2c
        self._i2c_address = _ADDRESS_ACCEL
        self._BUFFER1 = memoryview(self._BUFFER)[:1]
        self._BUFFER2 = memoryview(self._BUFFER)[:2]
        self._BUFFER6 = memoryview(self._BUFFER)[:6]

        self._set_register(_REG_ACCEL_CTRL_REG1_A, 0b_00000111) # self._enable_xyz = 0b111
        self._set_register(_REG_ACCEL_CTRL_REG2_A, 0)
        self._set_register(_REG_ACCEL_CTRL_REG3_A, 0)
        self._set_register(_REG_ACCEL_CTRL_REG4_A, 0b_10000000) # self._bdu = True
        self._set_register(_REG_ACCEL_CTRL_REG5_A, 0)
        self._set_register(_REG_ACCEL_CTRL_REG6_A, 0)

        self.data_rate = Rate.RATE_10_HZ
        self.mode = Mode.MODE_NORMAL
        self.range = Range.RANGE_2G

    def set_tap(
        self,
        tap: int, # Literal[0, 1, 2],
        threshold: int,
        *,
        time_limit: int = 10,
        time_latency: int = 20,
        time_window: int = 255,
        tap_cfg = None, #: Optional[int] = None,
    ) -> None:
        """The tap detection parameters.

        :param int tap: 0 to disable tap detection, 1 to detect only single taps, and 2 to detect \
            only double taps.
        :param int threshold: A threshold for the tap detection.  The higher the value the less\
            sensitive the detection. This changes based on the accelerometer range.  Good values\
            are 5-10 for 16G, 10-20 for 8G, 20-40 for 4G, and 40-80 for 2G.
        :param int time_limit: TIME_LIMIT register value. Defaults to :const:`10`
        :param int time_latency: TIME_LATENCY register value. Defaults to :const:`20`
        :param int time_window: TIME_WINDOW register value. Defaults to :const:`255`
        :param int tap_cfg: CLICK_CFG register value. Defaults to `None`
        """

        if tap not in (0, 1, 2) and tap_cfg is None:
            raise ValueError(
                "Tap must be 0 (disabled), 1 (single tap), or 2 (double tap)!"
            )
        if threshold > 127 or threshold < 0:
            raise ValueError("Threshold out of range (0-127)")

        if tap == 0 and tap_cfg is None:
            # Disable click interrupt.
            self._set_register(_REG_ACCEL_CTRL_REG3_A, 0, mask=1<<7) # self._tap_interrupt_enable = False
            self._set_register(_REG_ACCEL_CLICK_CFG_A, 0) # self._tap_config = 0
            return

        self._set_register(_REG_ACCEL_CTRL_REG3_A, -1, mask=1<<7) # self._tap_interrupt_enable = True

        if tap_cfg is None:
            if tap == 1:
                tap_cfg = 0x15  # Turn on all axes & singletap.
            if tap == 2:
                tap_cfg = 0x2A  # Turn on all axes & doubletap.
        # Or, if a custom tap configuration register value specified, use it.
        self._set_register(_REG_ACCEL_CLICK_CFG_A, tap_cfg, mask=0b_11111111) # self._tap_config = tap_cfg

        self._set_register(_REG_ACCEL_CLICK_THS_A, threshold) # self._tap_threshold = threshold  # why and?
        self._set_register(_REG_ACCEL_TIME_LIMIT_A, limit) # self._tap_time_limit = time_limit
        self._set_register(_REG_ACCEL_TIME_LATENCY_A, time_latency) # self._tap_time_latency = time_latency
        self._set_register(_REG_ACCEL_TIME_WINDOW_A, time_window) # self._tap_time_window = time_window

    @property
    def tapped(self) -> bool:
        """True if a tap was detected recently. Whether its a single tap or double tap is
        determined by the tap param on :meth:`set_tap`. :attr:`tapped` may be True over
        multiple reads even if only a single tap or single double tap occurred.
        """
        tap_src = self._get_register(_REG_ACCEL_CLICK_SRC_A) # tap_src = self._tap_src
        return tap_src & 0b1000000 > 0

    def _set_int(interrupt: int, cfg: int, threshold: int, duration: int, *, latch: bool = True, d4d: bool = False) -> None:
        if cfg < 0 or cfg > 255:
            raise AttributeError("cfg out of range (0-255)")
        if threshold < 0 or threshold > 127:
            raise AttributeError("Threshold out of range (0-127)")
        if duration < 0 or duration > 127:
            raise AttributeError("Duration out of range (0-127)")
        if interrupt == 1:
            self._set_register(_REG_ACCEL_INT1_CFG_A, cfg)
            self._set_register(_REG_ACCEL_INT1_THS_A, threshold)
            self._set_register(_REG_ACCEL_INT1_DURATION_A, duration)
            self._set_register(_REG_ACCEL_CTRL_REG5_A, -1 if latch else 0, mask=0b_00001000)
            self._set_register(_REG_ACCEL_CTRL_REG5_A, -1 if d4d   else 0, mask=0b_00000100)
        elif interrupt == 2:
            self._set_register(_REG_ACCEL_INT2_CFG_A, cfg)
            self._set_register(_REG_ACCEL_INT2_THS_A, threshold)
            self._set_register(_REG_ACCEL_INT2_DURATION_A, duration)
            self._set_register(_REG_ACCEL_CTRL_REG5_A, -1 if latch else 0, mask=0b_00000010)
            self._set_register(_REG_ACCEL_CTRL_REG5_A, -1 if d4d   else 0, mask=0b_00000001)
        else:
            raise ValueError('Interrupt must be 1 or 2')

    def set_int1(cfg: int, threshold: int, duration: int) -> None:
        self._set_int(1, cfg, src, threshold, duration)

    def set_int2(cfg: int, src: int, threshold: int, duration: int) -> None:
        self._set_int(2, cfg, src, threshold, duration)

    def set_interrupt_active(hl) -> None:
        self._set_register(_REG_ACCEL_CTRL_REG6_A, 0 if hl else -1, mask=0b_00000010) # register bit is 0 for active-high

    @property
    def interrupted1(self) -> int:
        return self._get_register(_REG_ACCEL_INT1_SRC_A)

    @property
    def interrupted2(self) -> int:
        return self._get_register(_REG_ACCEL_INT2_SRC_A)

    @property
    def _raw_acceleration(self) -> tuple: # Tuple[int, int, int]:
        self._read_bytes(_REG_ACCEL_OUT_X_L_A | 0x80, self._BUFFER6)
        return struct.unpack_from("<hhh", self._BUFFER6)

    @property
    def acceleration(self) -> tuple: # Tuple[float, float, float]:
        """The measured accelerometer sensor values.
        A 3-tuple of X, Y, Z axis values in m/s^2 squared that are signed floats.
        """

        raw_accel_data = self._raw_acceleration

        x = self._scale_data(raw_accel_data[0])
        y = self._scale_data(raw_accel_data[1])
        z = self._scale_data(raw_accel_data[2])

        return (x, y, z)

    def _scale_data(self, raw_measurement: int) -> float:
        lsb, shift = self._lsb_shift()

        return (raw_measurement >> shift) * lsb * _SMOLLER_GRAVITY

    def _lsb_shift(self) -> tuple: # Tuple[float, int]:  # pylint:disable=too-many-branches
        # the bit depth of the data depends on the mode, and the lsb value
        # depends on the mode and range
        lsb = -1  # the default, normal mode @ 2G

        if self._cached_mode is Mode.MODE_HIGH_RESOLUTION:  # 12-bit
            shift = 4
            if self._cached_range is Range.RANGE_2G:
                lsb = 0.98
            elif self._cached_range is Range.RANGE_4G:
                lsb = 1.95
            elif self._cached_range is Range.RANGE_8G:
                lsb = 3.9
            elif self._cached_range is Range.RANGE_16G:
                lsb = 11.72
        elif self._cached_mode is Mode.MODE_NORMAL:  # 10-bit
            shift = 6
            if self._cached_range is Range.RANGE_2G:
                lsb = 3.9
            elif self._cached_range is Range.RANGE_4G:
                lsb = 7.82
            elif self._cached_range is Range.RANGE_8G:
                lsb = 15.63
            elif self._cached_range is Range.RANGE_16G:
                lsb = 46.9

        elif self._cached_mode is Mode.MODE_LOW_POWER:  # 8-bit
            shift = 8
            if self._cached_range is Range.RANGE_2G:
                lsb = 15.63
            elif self._cached_range is Range.RANGE_4G:
                lsb = 31.26
            elif self._cached_range is Range.RANGE_8G:
                lsb = 62.52
            elif self._cached_range is Range.RANGE_16G:
                lsb = 187.58

        if lsb is -1:
            raise AttributeError(
                "'impossible' range or mode detected: "
                f"range: {self._cached_range} mode: {self._cached_mode}"
            )
        return (lsb, shift)

    @property
    def data_rate(self) -> int:
        """Select the rate at which the sensor takes measurements. Must be a `Rate`"""
        return self._cached_data_rate

    @data_rate.setter
    def data_rate(self, value: int) -> None:
        if value not in (
            Rate.RATE_SHUTDOWN,
            Rate.RATE_1_HZ,
            Rate.RATE_10_HZ,
            Rate.RATE_25_HZ,
            Rate.RATE_50_HZ,
            Rate.RATE_100_HZ,
            Rate.RATE_200_HZ,
            Rate.RATE_400_HZ,
            Rate.RATE_1620_HZ,
            Rate.RATE_1344_HZ,
            Rate.RATE_5376_HZ,
        ):
            raise AttributeError("data_rate must be a `Rate`")
        if value == Rate.RATE_5376_HZ:
            value = Rate.RATE_1344_HZ # = 9
        self._set_register(_REG_ACCEL_CTRL_REG1_A, value << 4, mask=0b_11110000)
        self._cached_data_rate = value

    @property
    def range(self) -> int:
        """Adjusts the range of values that the sensor can measure, from +- 2G to +-16G
        Note that larger ranges will be less accurate. Must be a `Range`"""
        return self._cached_range

    @range.setter
    def range(self, value: int) -> None:
        if value not in (
            Range.RANGE_2G,
            Range.RANGE_4G,
            Range.RANGE_8G,
            Range.RANGE_16G,
        ):
            raise AttributeError("range must be a `Range`")
        self._set_register(_REG_ACCEL_CTRL_REG4_A, value << 4, mask=0b_00110000)
        self._cached_range = value

    @property
    def mode(self) -> int:
        """Sets the power mode of the sensor. The mode must be a `Mode`. Note that the
        mode and range will both affect the accuracy of the sensor"""
        return self._cached_mode

    @mode.setter
    def mode(self, value: int) -> None:
        if value not in (
            Mode.MODE_NORMAL,
            Mode.MODE_HIGH_RESOLUTION,
            Mode.MODE_LOW_POWER,
        ):
            raise AttributeError("mode must be a `Mode`")
        self._set_register(_REG_ACCEL_CTRL_REG4_A, -1 if (value & 0b01) else 0, mask=0b_00001000) # high resolution
        self._set_register(_REG_ACCEL_CTRL_REG1_A, -1 if (value & 0b10) else 0, mask=0b_00001000) # low power
        self._cached_mode = value

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



"""
# Accelerometer registers -- names follow CircuitPython
_REG_ACCEL_WHO_AM_I  = const(0x0F) # LSM303AGR only
_REG_ACCEL_ACT_THS_A = const(0x3E) # LSM303AGR only
_REG_ACCEL_ACT_DUR_A = const(0x3F) # LSM303AGR only

class LSM303AGR_Accel(LSM303DLHC_Accel):
    @property
    def whoami(self) -> int:
        return self._get_register(_REG_ACCEL_WHO_AM_I)

    @property
    def _chip_id(self) -> int:
        # CircuitPython compatibility
        return self.whoami

    def inactivity(self, threshold: int, duration: int):
        if threshold < 0 or threshold > 127:
            raise AttributeError("Threshold out of range (0-127)")
        if duration < 0 or duration > 255:
            raise AttributeError("Duration out of range (0-255)")
        self._set_register(_REG_ACCEL_ACT_THS_A, threshold, mask=0b_01111111)
        self._set_register(_REG_ACCEL_ACT_DUR_A, duration)
"""