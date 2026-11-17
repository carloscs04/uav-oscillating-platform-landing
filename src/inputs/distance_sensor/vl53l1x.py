# File: src/inputs/distance_sensor/vl53l1x.py
"""
Minimal VL53L1X time-of-flight driver (VL53L0/1XV2 board) over I2C with smbus2.

Port of ST's VL53L1X Ultra Lite Driver (ULD, BSD-3) sequence: default
configuration block, distance mode, timing budget, continuous ranging.
No extra package needed beyond smbus2.

    with VL53L1X(bus=1) as tof:
        tof.start_ranging()
        mm = tof.read_mm()          # None = no valid target this sample

Wiring (Pi 5): VIN->pin 1 (3.3 V), GND->pin 6, SDA->pin 3, SCL->pin 5.
Range: "short" mode up to ~1.3 m (robust in daylight), "long" up to ~4 m
(dark) / ~2 m+ indoors. Field of view about 27 deg.
"""

import time
from typing import Optional

from smbus2 import SMBus, i2c_msg

ADDRESS = 0x29
MODEL_ID = 0xEACC

SOFT_RESET = 0x0000
VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND = 0x0008
GPIO_HV_MUX__CTRL = 0x0030
GPIO__TIO_HV_STATUS = 0x0031
PHASECAL_CONFIG__TIMEOUT_MACROP = 0x004B
RANGE_CONFIG__TIMEOUT_MACROP_A_HI = 0x005E
RANGE_CONFIG__VCSEL_PERIOD_A = 0x0060
RANGE_CONFIG__TIMEOUT_MACROP_B_HI = 0x0061
RANGE_CONFIG__VCSEL_PERIOD_B = 0x0063
RANGE_CONFIG__VALID_PHASE_HIGH = 0x0069
SYSTEM__INTERMEASUREMENT_PERIOD = 0x006C
SD_CONFIG__WOI_SD0 = 0x0078
SD_CONFIG__INITIAL_PHASE_SD0 = 0x007A
SYSTEM__INTERRUPT_CLEAR = 0x0086
SYSTEM__MODE_START = 0x0087
RESULT__RANGE_STATUS = 0x0089
RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0 = 0x0096
RESULT__OSC_CALIBRATE_VAL = 0x00DE
FIRMWARE__SYSTEM_STATUS = 0x00E5
IDENTIFICATION__MODEL_ID = 0x010F

# ST ULD VL51L1X_DEFAULT_CONFIGURATION, written to registers 0x2D..0x87.
_DEFAULT_CONFIG = bytes([
    0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x02, 0x08, 0x00, 0x08, 0x10, 0x01, 0x01, 0x00,
    0x00, 0x00, 0x00, 0xFF, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x0B, 0x00,
    0x00, 0x02, 0x0A, 0x21, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xC8, 0x00, 0x00,
    0x38, 0xFF, 0x01, 0x00, 0x08, 0x00, 0x00, 0x01, 0xCC, 0x0F, 0x01, 0xF1, 0x0D, 0x01,
    0x68, 0x00, 0x80, 0x08, 0xB8, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x89, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x01, 0x0F, 0x0D, 0x0E, 0x0E, 0x00, 0x00, 0x02, 0xC7, 0xFF,
    0x9B, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,
])
assert len(_DEFAULT_CONFIG) == 0x87 - 0x2D + 1

# Distance mode registers: (phasecal timeout, vcsel A, vcsel B, valid phase high, WOI, initial phase)
_MODES = {
    "short": (0x14, 0x07, 0x05, 0x38, 0x0705, 0x0606),
    "long":  (0x0A, 0x0F, 0x0D, 0xB8, 0x0F0D, 0x0E0E),
}
# Timing budget (ms) -> (TIMEOUT_MACROP_A, TIMEOUT_MACROP_B), per distance mode (ST ULD tables).
_BUDGETS = {
    "short": {15: (0x001D, 0x0027), 20: (0x0051, 0x006E), 33: (0x00D6, 0x006E),
              50: (0x01AE, 0x01E8), 100: (0x02E1, 0x0388), 200: (0x03E1, 0x0496),
              500: (0x0591, 0x05C1)},
    "long":  {20: (0x001E, 0x0022), 33: (0x0060, 0x006E), 50: (0x00AD, 0x00C6),
              100: (0x01CC, 0x01EA), 200: (0x02D9, 0x02F8), 500: (0x048F, 0x04A4)},
}
# Raw RESULT__RANGE_STATUS -> ST range status (0 = valid). 255 = undefined.
_STATUS_RTN = (255, 255, 255, 5, 2, 4, 1, 7, 3, 0, 255, 255, 9, 13, 255, 255,
               255, 255, 10, 6, 255, 255, 11, 12)
STATUS_TEXT = {0: "valid", 1: "sigma fail", 2: "signal fail", 4: "out of bounds",
               7: "wrap-around", 255: "no target"}


class VL53L1XError(IOError):
    pass


class VL53L1X:
    def __init__(self, bus: int = 1, address: int = ADDRESS, distance_mode: str = "long",
                 timing_budget_ms: int = 50, period_ms: Optional[int] = None,
                 io_timeout_s: float = 0.5):
        self.address = address
        self.io_timeout_s = io_timeout_s
        self.bus = SMBus(bus)
        self.last_status = 255
        try:
            self._init_sensor()
            self.configure(distance_mode, timing_budget_ms, period_ms)
        except Exception:
            self.close()
            raise

    # ------------------------------------------------- register io (16-bit)
    def _write(self, reg, data):
        self.bus.i2c_rdwr(i2c_msg.write(self.address, [reg >> 8, reg & 0xFF, *data]))

    def _read(self, reg, n):
        read = i2c_msg.read(self.address, n)
        self.bus.i2c_rdwr(i2c_msg.write(self.address, [reg >> 8, reg & 0xFF]), read)
        return bytes(read)

    def _w8(self, reg, val):
        self._write(reg, [val & 0xFF])

    def _w16(self, reg, val):
        self._write(reg, [(val >> 8) & 0xFF, val & 0xFF])

    def _w32(self, reg, val):
        self._write(reg, list(int(val).to_bytes(4, "big")))

    def _r8(self, reg):
        return self._read(reg, 1)[0]

    def _r16(self, reg):
        return int.from_bytes(self._read(reg, 2), "big")

    def _wait(self, cond, what):
        deadline = time.monotonic() + self.io_timeout_s
        while not cond():
            if time.monotonic() > deadline:
                raise VL53L1XError(f"VL53L1X timeout waiting for {what}")
            time.sleep(0.002)

    # ------------------------------------------------------------------ init
    def _init_sensor(self):
        model = self._r16(IDENTIFICATION__MODEL_ID)
        if model != MODEL_ID:
            raise VL53L1XError(f"device at 0x{self.address:02x} is not a VL53L1X "
                               f"(model id 0x{model:04x}, expected 0x{MODEL_ID:04x})")
        # soft reset, then wait for the firmware to boot
        self._w8(SOFT_RESET, 0x00)
        time.sleep(0.001)
        self._w8(SOFT_RESET, 0x01)
        time.sleep(0.002)
        self._wait(lambda: self._r8(FIRMWARE__SYSTEM_STATUS) & 0x01, "boot")

        self._write(0x2D, _DEFAULT_CONFIG)
        # one dummy measurement completes the VHV calibration
        self.start_ranging()
        self._wait(self.data_ready, "first measurement")
        self._w8(SYSTEM__INTERRUPT_CLEAR, 0x01)
        self.stop_ranging()
        self._w8(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x09)   # two VHV bounds
        self._w8(0x000B, 0x00)                                  # start VHV from previous temp

    def configure(self, distance_mode="long", timing_budget_ms=50, period_ms=None):
        """distance_mode: 'short' (<=1.3 m, best in daylight) or 'long' (<=4 m).
        timing_budget_ms: time per measurement (one of the ST table values).
        period_ms: time between measurements (>= budget); default = budget."""
        if distance_mode not in _MODES:
            raise ValueError(f"distance_mode must be one of {list(_MODES)}")
        table = _BUDGETS[distance_mode]
        if timing_budget_ms not in table:
            raise ValueError(f"timing budget for '{distance_mode}' must be one of {sorted(table)} ms")
        period_ms = max(int(period_ms or timing_budget_ms), timing_budget_ms)

        phasecal, vcsel_a, vcsel_b, phase_high, woi, init_phase = _MODES[distance_mode]
        self._w8(PHASECAL_CONFIG__TIMEOUT_MACROP, phasecal)
        self._w8(RANGE_CONFIG__VCSEL_PERIOD_A, vcsel_a)
        self._w8(RANGE_CONFIG__VCSEL_PERIOD_B, vcsel_b)
        self._w8(RANGE_CONFIG__VALID_PHASE_HIGH, phase_high)
        self._w16(SD_CONFIG__WOI_SD0, woi)
        self._w16(SD_CONFIG__INITIAL_PHASE_SD0, init_phase)

        a, b = table[timing_budget_ms]
        self._w16(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, a)
        self._w16(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, b)

        clock_pll = self._r16(RESULT__OSC_CALIBRATE_VAL) & 0x03FF
        self._w32(SYSTEM__INTERMEASUREMENT_PERIOD, int(clock_pll * period_ms * 1.075))

        self.distance_mode, self.timing_budget_ms, self.period_ms = \
            distance_mode, timing_budget_ms, period_ms

    # --------------------------------------------------------------- ranging
    def start_ranging(self):
        self._w8(SYSTEM__INTERRUPT_CLEAR, 0x01)
        self._w8(SYSTEM__MODE_START, 0x40)

    def stop_ranging(self):
        self._w8(SYSTEM__MODE_START, 0x00)

    def data_ready(self) -> bool:
        polarity = 0 if self._r8(GPIO_HV_MUX__CTRL) & 0x10 else 1   # 1 = active high
        return (self._r8(GPIO__TIO_HV_STATUS) & 0x01) == polarity

    def read_mm(self) -> Optional[int]:
        """Wait for the next sample. Returns millimetres, or None when the sample is
        not valid (no target / too far / too weak; see last_status).
        Raises VL53L1XError / OSError if the sensor stops answering."""
        self._wait(self.data_ready, "range sample")
        raw = self._r8(RESULT__RANGE_STATUS) & 0x1F
        mm = self._r16(RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0)
        self._w8(SYSTEM__INTERRUPT_CLEAR, 0x01)
        self.last_status = _STATUS_RTN[raw] if raw < len(_STATUS_RTN) else 255
        return mm if self.last_status == 0 else None

    def close(self):
        try:
            self.stop_ranging()
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
    # Quick check:  python3 src/inputs/distance_sensor/vl53l1x.py [short|long]
    import sys
    mode = sys.argv[1] if len(sys.argv) > 1 else "long"
    with VL53L1X(distance_mode=mode) as tof:
        print(f"VL53L1X ready: {tof.distance_mode} mode, budget {tof.timing_budget_ms} ms, "
              f"period {tof.period_ms} ms. Ctrl+C to stop.")
        tof.start_ranging()
        t0, n = time.monotonic(), 0
        try:
            while True:
                mm = tof.read_mm()
                n += 1
                rate = n / (time.monotonic() - t0)
                text = f"{mm:5d} mm" if mm is not None else \
                    f"  ---    ({STATUS_TEXT.get(tof.last_status, tof.last_status)})"
                print(f"{text}   [{rate:4.1f} Hz]")
        except KeyboardInterrupt:
            pass
