# Raw magnetometer data (is it accessible over SPI?)
# DPM Data
# Incorporate in Versatile Framework

from .ICM20948 import ICM20948

imu = ICM20948()
print(imu.measure())
