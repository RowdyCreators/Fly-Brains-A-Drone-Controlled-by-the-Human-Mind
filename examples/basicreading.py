import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import time

i2c = busio.I2C(board.SCL, board.SDA)
ads = ADS.ADS1115(i2c)
channel = AnalogIn(ads, 0)

while True:
    voltage = channel.voltage
    bar_length = int((voltage / 3.3) * 50)
    bar = "█" * bar_length
    print(f"\r{voltage:.4f}V |{bar:<50}|", end="")
    time.sleep(0.01)
