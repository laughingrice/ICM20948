# Raw magnetometer data (is it accessible over SPI?)
# DPM Data
# Incorporate in Versatile Framework

import time
from .ICM20948 import ICM20948

imu = ICM20948()

for i in range(10):
    try:
        imu._setup()
    except Exception as e:
        print('got exception: {} - trying again'.format(e.args[0]))

    if imu.AK09916_initialized:
        break


# Print out reading every second
while True:
    print('  '.join(['{:> 7.3}'.format(float(x)) for x in imu.measure()]))
    time.sleep(1)
