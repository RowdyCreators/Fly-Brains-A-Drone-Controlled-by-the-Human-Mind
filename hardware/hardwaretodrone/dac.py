"""MCP4728 4-channel 12-bit DAC driver (smbus2).

Only uses commands that write the volatile DAC registers, never EEPROM,
so it's safe to call at control-loop rates.
"""

MAX_CODE = 4095


class MCP4728:
    def __init__(self, bus: int = 1, address: int = 0x60):
        from smbus2 import SMBus           # imported here so FakeDAC works without it
        self.address = address
        self._bus = SMBus(bus)

    def set_codes(self, codes):
        """Fast Write: update all four channels in one I2C transaction.

        Per channel: [0 0 PD1 PD0 D11 D10 D9 D8] [D7..D0], PD=00 (on).
        Vref/gain stay at the chip's power-on defaults (Vref = VDD, gain x1).
        """
        if len(codes) != 4:
            raise ValueError("need exactly 4 codes (A, B, C, D)")
        data = []
        for c in codes:
            c = max(0, min(MAX_CODE, int(c)))
            data += [(c >> 8) & 0x0F, c & 0xFF]
        # smbus2 sends: address, data[0] (as "register"), data[1:]
        self._bus.write_i2c_block_data(self.address, data[0], data[1:])

    def set_code(self, channel: int, code: int):
        """Multi-Write: update one channel, forcing Vref = VDD and gain x1."""
        if not 0 <= channel <= 3:
            raise ValueError("channel must be 0-3")
        code = max(0, min(MAX_CODE, int(code)))
        cmd = 0x40 | (channel << 1)                    # 0 1 0 0 0 DAC1 DAC0 UDAC=0
        self._bus.write_i2c_block_data(self.address, cmd,
                                       [(code >> 8) & 0x0F, code & 0xFF])

    def read_codes(self):
        """Read back the live DAC register for each channel (for debugging)."""
        from smbus2 import i2c_msg
        msg = i2c_msg.read(self.address, 24)
        self._bus.i2c_rdwr(msg)
        raw = list(msg)
        return [((raw[i * 6 + 1] & 0x0F) << 8) | raw[i * 6 + 2] for i in range(4)]

    def close(self):
        self._bus.close()


class FakeDAC:
    """Stand-in for testing without hardware. Records every write."""

    def __init__(self, *_, verbose=False, **__):
        self.codes = [2048] * 4
        self.writes = 0
        self.verbose = verbose

    def set_codes(self, codes):
        self.codes = [max(0, min(MAX_CODE, int(c))) for c in codes]
        self.writes += 1
        if self.verbose:
            print("DAC", " ".join(f"{'ABCD'[i]}={c:4d}" for i, c in enumerate(self.codes)))

    def set_code(self, channel, code):
        self.codes[channel] = max(0, min(MAX_CODE, int(code)))
        self.writes += 1

    def read_codes(self):
        return list(self.codes)

    def close(self):
        pass
