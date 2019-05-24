# Raw magnetometer data (is it accessible over SPI?)
# DPM Data
# Incorporate in Versatile Framework

import time
from .ICM20948 import ICM20948

imu = ICM20948()

# Print out reading every second
while True:
    print('  '.join(['{:> 7.3}'.format(float(x)) for x in imu.measure()]))
    time.sleep(1)
