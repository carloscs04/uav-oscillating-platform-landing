# File: src/inputs/distance_sensor/vl53l0x.py
"""
Minimal VL53L0X time-of-flight driver (GY-VL53L0XV2 board) over I2C with smbus2.

Port of the init / timing / continuous-ranging sequence of Pololu's VL53L0X
Arduino library (MIT, https://github.com/pololu/vl53l0x-arduino), which in turn
follows ST's VL53L0X API. No extra package needed beyond smbus2.

    with VL53L0X(bus=1) as tof:
        tof.start_continuous()
        mm = tof.read_mm()          # None = no target in range

Wiring (Pi 5): VIN->pin 1 (3.3 V), GND->pin 6, SDA->pin 3, SCL->pin 5.!!!
Default range mode: about 30 mm .. 1200 mm (up to ~2 m on bright white targets).
"""

import time
from typing import Optional

from smbus2 import SMBus, i2c_msg

ADDRESS = 0x29
MODEL_ID = 0xEE

SYSRANGE_START = 0x00
SYSTEM_SEQUENCE_CONFIG = 0x01
SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0A
SYSTEM_INTERRUPT_CLEAR = 0x0B
RESULT_INTERRUPT_STATUS = 0x13
RESULT_RANGE_STATUS = 0x14
FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44
MSRC_CONFIG_TIMEOUT_MACROP = 0x46
PRE_RANGE_CONFIG_VCSEL_PERIOD = 0x50
PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x51
MSRC_CONFIG_CONTROL = 0x60
FINAL_RANGE_CONFIG_VCSEL_PERIOD = 0x70
FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x71
GPIO_HV_MUX_ACTIVE_HIGH = 0x84
VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV = 0x89
GLOBAL_CONFIG_SPAD_ENABLES_REF_0 = 0xB0
GLOBAL_CONFIG_REF_EN_START_SELECT = 0xB6
DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD = 0x4E
DYNAMIC_SPAD_REF_EN_START_OFFSET = 0x4F
IDENTIFICATION_MODEL_ID = 0xC0

OUT_OF_RANGE_MM = 8190          # the sensor reports 8190/8191 when nothing is in range

# ST's default tuning settings (register, value), applied in order.
_TUNING = [
    (0xFF, 0x01), (0x00, 0x00), (0xFF, 0x00), (0x09, 0x00), (0x10, 0x00), (0x11, 0x00),
    (0x24, 0x01), (0x25, 0xFF), (0x75, 0x00), (0xFF, 0x01), (0x4E, 0x2C), (0x48, 0x00),
    (0x30, 0x20), (0xFF, 0x00), (0x30, 0x09), (0x54, 0x00), (0x31, 0x04), (0x32, 0x03),
    (0x40, 0x83), (0x46, 0x25), (0x60, 0x00), (0x27, 0x00), (0x50, 0x06), (0x51, 0x00),
    (0x52, 0x96), (0x56, 0x08), (0x57, 0x30), (0x61, 0x00), (0x62, 0x00), (0x64, 0x00),
    (0x65, 0x00), (0x66, 0xA0), (0xFF, 0x01), (0x22, 0x32), (0x47, 0x14), (0x49, 0xFF),
    (0x4A, 0x00), (0xFF, 0x00), (0x7A, 0x0A), (0x7B, 0x00), (0x78, 0x21), (0xFF, 0x01),
    (0x23, 0x34), (0x42, 0x00), (0x44, 0xFF), (0x45, 0x26), (0x46, 0x05), (0x40, 0x40),
    (0x0E, 0x06), (0x20, 0x1A), (0x43, 0x40), (0xFF, 0x00), (0x34, 0x03), (0x35, 0x44),
    (0xFF, 0x01), (0x31, 0x04), (0x4B, 0x09), (0x4C, 0x05), (0x4D, 0x04), (0xFF, 0x00),
    (0x44, 0x00), (0x45, 0x20), (0x47, 0x08), (0x48, 0x28), (0x67, 0x00), (0x70, 0x04),
    (0x71, 0x01), (0x72, 0xFE), (0x76, 0x00), (0x77, 0x00), (0xFF, 0x01), (0x0D, 0x01),
    (0xFF, 0x00), (0x80, 0x01), (0x01, 0xF8), (0xFF, 0x01), (0x8E, 0x01), (0x00, 0x01),
    (0xFF, 0x00), (0x80, 0x00),
]


class VL53L0XError(IOError):
    pass


def _decode_vcsel_period(reg: int) -> int:
    return (reg + 1) << 1


def _macro_period_ns(vcsel_period_pclks: int) -> int:
    return ((2304 * vcsel_period_pclks * 1655) + 500) // 1000


def _mclks_to_us(mclks: int, vcsel_period_pclks: int) -> int:
    mp = _macro_period_ns(vcsel_period_pclks)
    return ((mclks * mp) + (mp // 2)) // 1000


def _us_to_mclks(us: int, vcsel_period_pclks: int) -> int:
    mp = _macro_period_ns(vcsel_period_pclks)
    return ((us * 1000) + (mp // 2)) // mp


def _decode_timeout(reg: int) -> int:
    return ((reg & 0x00FF) << ((reg & 0xFF00) >> 8)) + 1


def _encode_timeout(mclks: int) -> int:
    if mclks <= 0:
        return 0
    ls, ms = mclks - 1, 0
    while ls & 0xFFFFFF00:
        ls >>= 1
        ms += 1
    return (ms << 8) | (ls & 0xFF)


class VL53L0X:
    def __init__(self, bus: int = 1, address: int = ADDRESS, timing_budget_us: int = 33000,
                 io_timeout_s: float = 0.5):
        self.address = address
        self.io_timeout_s = io_timeout_s
        self.bus = SMBus(bus)
        self._stop_variable = 0
        try:
            self._init_sensor()
            self.set_timing_budget(timing_budget_us)
        except Exception:
            self.close()
            raise

    # ----------------------------------------------------------- register io
    def _w(self, reg, val):
        self.bus.write_byte_data(self.address, reg, val & 0xFF)

    def _r(self, reg):
        return self.bus.read_byte_data(self.address, reg)

    def _w16(self, reg, val):
        self.bus.write_i2c_block_data(self.address, reg, [(val >> 8) & 0xFF, val & 0xFF])

    def _r16(self, reg):
        hi, lo = self.bus.read_i2c_block_data(self.address, reg, 2)
        return (hi << 8) | lo

    def _read_multi(self, reg, n):
        write, read = i2c_msg.write(self.address, [reg]), i2c_msg.read(self.address, n)
        self.bus.i2c_rdwr(write, read)
        return list(read)

    def _wait(self, cond, what):
        deadline = time.monotonic() + self.io_timeout_s
        while not cond():
            if time.monotonic() > deadline:
                raise VL53L0XError(f"VL53L0X timeout waiting for {what}")
            time.sleep(0.001)

    # ------------------------------------------------------------------ init
    def _init_sensor(self):
        model = self._r(IDENTIFICATION_MODEL_ID)
        if model != MODEL_ID:
            raise VL53L0XError(f"device at 0x{self.address:02x} is not a VL53L0X "
                               f"(model id 0x{model:02x}, expected 0x{MODEL_ID:02x})")

        # I/O at 2.8 V (GY board has a regulator + level shifting on 3.3 V)
        self._w(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, self._r(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01)

        # I2C standard mode, read stop variable
        for reg, val in ((0x88, 0x00), (0x80, 0x01), (0xFF, 0x01), (0x00, 0x00)):
            self._w(reg, val)
        self._stop_variable = self._r(0x91)
        for reg, val in ((0x00, 0x01), (0xFF, 0x00), (0x80, 0x00)):
            self._w(reg, val)

        # disable SIGNAL_RATE_MSRC and SIGNAL_RATE_PRE_RANGE limit checks
        self._w(MSRC_CONFIG_CONTROL, self._r(MSRC_CONFIG_CONTROL) | 0x12)
        # final range signal rate limit: 0.25 MCPS (fixed point 9.7)
        self._w16(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, int(0.25 * (1 << 7)))
        self._w(SYSTEM_SEQUENCE_CONFIG, 0xFF)

        # reference SPADs
        spad_count, is_aperture = self._get_spad_info()
        ref_map = self._read_multi(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, 6)
        for reg, val in ((0xFF, 0x01), (DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00),
                         (DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C), (0xFF, 0x00),
                         (GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4)):
            self._w(reg, val)
        first = 12 if is_aperture else 0
        enabled = 0
        for i in range(48):
            if i < first or enabled == spad_count:
                ref_map[i // 8] &= ~(1 << (i % 8)) & 0xFF
            elif (ref_map[i // 8] >> (i % 8)) & 0x1:
                enabled += 1
        self.bus.write_i2c_block_data(self.address, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_map)

        for reg, val in _TUNING:
            self._w(reg, val)

        # interrupt: new sample ready, active low
        self._w(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04)
        self._w(GPIO_HV_MUX_ACTIVE_HIGH, self._r(GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10 & 0xFF)
        self._w(SYSTEM_INTERRUPT_CLEAR, 0x01)

        budget = self.get_timing_budget()
        self._w(SYSTEM_SEQUENCE_CONFIG, 0xE8)       # disable MSRC and TCC by default
        self.set_timing_budget(budget)              # recalc for the new sequence steps

        # reference calibration: VHV then phase
        self._w(SYSTEM_SEQUENCE_CONFIG, 0x01)
        self._single_ref_calibration(0x40)
        self._w(SYSTEM_SEQUENCE_CONFIG, 0x02)
        self._single_ref_calibration(0x00)
        self._w(SYSTEM_SEQUENCE_CONFIG, 0xE8)

    def _get_spad_info(self):
        for reg, val in ((0x80, 0x01), (0xFF, 0x01), (0x00, 0x00), (0xFF, 0x06)):
            self._w(reg, val)
        self._w(0x83, self._r(0x83) | 0x04)
        for reg, val in ((0xFF, 0x07), (0x81, 0x01), (0x80, 0x01), (0x94, 0x6B), (0x83, 0x00)):
            self._w(reg, val)
        self._wait(lambda: self._r(0x83) != 0x00, "SPAD info")
        self._w(0x83, 0x01)
        tmp = self._r(0x92)
        for reg, val in ((0x81, 0x00), (0xFF, 0x06)):
            self._w(reg, val)
        self._w(0x83, self._r(0x83) & ~0x04 & 0xFF)
        for reg, val in ((0xFF, 0x01), (0x00, 0x01), (0xFF, 0x00), (0x80, 0x00)):
            self._w(reg, val)
        return tmp & 0x7F, bool((tmp >> 7) & 0x01)

    def _single_ref_calibration(self, vhv_init_byte):
        self._w(SYSRANGE_START, 0x01 | vhv_init_byte)
        self._wait(lambda: self._r(RESULT_INTERRUPT_STATUS) & 0x07, "reference calibration")
        self._w(SYSTEM_INTERRUPT_CLEAR, 0x01)
        self._w(SYSRANGE_START, 0x00)

    # ------------------------------------------------------- timing budget
    def _sequence_steps(self):
        sc = self._r(SYSTEM_SEQUENCE_CONFIG)
        en = {"tcc": (sc >> 4) & 1, "dss": (sc >> 3) & 1, "msrc": (sc >> 2) & 1,
              "pre_range": (sc >> 6) & 1, "final_range": (sc >> 7) & 1}

        pre_vcsel = _decode_vcsel_period(self._r(PRE_RANGE_CONFIG_VCSEL_PERIOD))
        t = {"pre_vcsel": pre_vcsel}
        msrc_mclks = self._r(MSRC_CONFIG_TIMEOUT_MACROP) + 1
        t["msrc_dss_tcc_us"] = _mclks_to_us(msrc_mclks, pre_vcsel)
        t["pre_range_mclks"] = _decode_timeout(self._r16(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI))
        t["pre_range_us"] = _mclks_to_us(t["pre_range_mclks"], pre_vcsel)

        final_vcsel = _decode_vcsel_period(self._r(FINAL_RANGE_CONFIG_VCSEL_PERIOD))
        t["final_vcsel"] = final_vcsel
        final_mclks = _decode_timeout(self._r16(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI))
        if en["pre_range"]:
            final_mclks -= t["pre_range_mclks"]
        t["final_range_us"] = _mclks_to_us(final_mclks, final_vcsel)
        return en, t

    @staticmethod
    def _overhead(en, t):
        used = 1910 + 960                                      # start + end
        if en["tcc"]:
            used += t["msrc_dss_tcc_us"] + 590
        if en["dss"]:
            used += 2 * (t["msrc_dss_tcc_us"] + 690)
        elif en["msrc"]:
            used += t["msrc_dss_tcc_us"] + 660
        if en["pre_range"]:
            used += t["pre_range_us"] + 660
        return used

    def get_timing_budget(self) -> int:
        en, t = self._sequence_steps()
        budget = self._overhead(en, t)
        if en["final_range"]:
            budget += t["final_range_us"] + 550
        return budget

    def set_timing_budget(self, budget_us: int):
        """Time per measurement: 20 ms (fast, noisier) .. 200 ms (slow, precise). Default ~33 ms."""
        if budget_us < 20000:
            raise ValueError("VL53L0X timing budget must be >= 20000 us")
        en, t = self._sequence_steps()
        used = self._overhead(en, t)
        if en["final_range"]:
            used += 550
            if used > budget_us:
                raise ValueError(f"timing budget {budget_us} us too short")
            mclks = _us_to_mclks(budget_us - used, t["final_vcsel"])
            if en["pre_range"]:
                mclks += t["pre_range_mclks"]
            self._w16(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, _encode_timeout(mclks))
        self.timing_budget_us = budget_us

    # --------------------------------------------------------------- ranging
    def start_continuous(self):
        """Back-to-back ranging: a new sample every timing budget."""
        for reg, val in ((0x80, 0x01), (0xFF, 0x01), (0x00, 0x00), (0x91, self._stop_variable),
                         (0x00, 0x01), (0xFF, 0x00), (0x80, 0x00)):
            self._w(reg, val)
        self._w(SYSRANGE_START, 0x02)

    def stop_continuous(self):
        for reg, val in ((SYSRANGE_START, 0x01), (0xFF, 0x01), (0x00, 0x00), (0x91, 0x00),
                         (0x00, 0x01), (0xFF, 0x00)):
            self._w(reg, val)

    def read_mm(self) -> Optional[int]:
        """Wait for the next continuous sample. Returns millimetres, or None when no
        target is in range. Raises VL53L0XError / OSError if the sensor stops answering."""
        self._wait(lambda: self._r(RESULT_INTERRUPT_STATUS) & 0x07, "range sample")
        mm = self._r16(RESULT_RANGE_STATUS + 10)
        self._w(SYSTEM_INTERRUPT_CLEAR, 0x01)
        return None if mm >= OUT_OF_RANGE_MM else mm

    def close(self):
        try:
            self.stop_continuous()
        except Exception:
            pass
        try:
            self.bus.close()
        except Exception:
            pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


if __name__ == "__main__":
    # Quick check:  python3 src/inputs/distance_sensor/vl53l0x.py
    with VL53L0X() as tof:
        print(f"VL53L0X ready, timing budget {tof.timing_budget_us} us. Ctrl+C to stop.")
        tof.start_continuous()
        try:
            while True:
                mm = tof.read_mm()
                print("out of range" if mm is None else f"{mm} mm")
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
