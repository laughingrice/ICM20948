# TODO: Read raw acc, gyro and compass data
# TODO: setup DPM
# TODO: read DPM Data
# TODO: setup FIFO
# TODO: work into Versatile framework

from .ICM20948 import ICM20948

imu = ICM20948()

print(imu.measure())
