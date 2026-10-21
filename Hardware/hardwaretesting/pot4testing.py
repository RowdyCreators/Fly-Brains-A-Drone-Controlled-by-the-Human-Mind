"""Raw manual control of channel D (POT 4 / throttle) only. No library.

    python3 pot4.py

Commands (type, then Enter):
    u        full UP     (3.30V)
    d        full DOWN   (0V)
    c        center      (1.65V)
    b        bind: UP 1s, DOWN 1s, center
    1.2      any voltage 0-3.3
    q        quit (centers first)
"""

import time
from smbus2 import SMBus, i2c_msg

ADDR, VDD = 0x60, 3.3
bus = SMBus(1)


def write(ch, volts):
    code = max(0, min(4095, round(volts / VDD * 4095)))
    bus.write_i2c_block_data(ADDR, 0x40 | (ch << 1), [code >> 8, code & 0xFF])
    msg = i2c_msg.read(ADDR, 24)
    bus.i2c_rdwr(msg)
    r = list(msg)[ch * 6: ch * 6 + 3]
    readback = ((r[1] & 0x0F) << 8) | r[2]
    print(f"  D -> {volts:.2f}V  (code {code}, chip reports {readback})")


for ch in range(3):          # A, B, C held at center
    bus.write_i2c_block_data(ADDR, 0x40 | (ch << 1), [0x08, 0x00])
write(3, 1.65)

print(__doc__)
try:
    while True:
        cmd = input("pot4> ").strip().lower()
        if cmd == "q":
            break
        elif cmd == "u":
            write(3, VDD)
        elif cmd == "d":
            write(3, 0.0)
        elif cmd == "c":
            write(3, 1.65)
        elif cmd == "b":
            print("bind: UP");   write(3, VDD);  time.sleep(1)
            print("bind: DOWN"); write(3, 0.0);  time.sleep(1)
            print("bind: CENTER"); write(3, 1.65)
        else:
            try:
                write(3, float(cmd))
            except ValueError:
                print("  u / d / c / b / a voltage / q")
except (KeyboardInterrupt, EOFError):
    pass
finally:
    write(3, 1.65)
    bus.close()
    print("Centered, closed.")