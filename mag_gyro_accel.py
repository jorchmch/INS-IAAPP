# SPDX-FileCopyrightText: Copyright (c) 2020 Bryan Siepert for Adafruit Industries
#
# SPDX-License-Identifier: MIT
import time
import board
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX
from adafruit_lis3mdl import LIS3MDL, Range

i2c = board.I2C()  # uses board.SCL and board.SDA
sensor = LSM6DSOX(i2c)
magne = LIS3MDL(i2c)

magne.range = Range.RANGE_4_GAUSS

while True:
    mag_x, mag_y, mag_z = magne.magnetic

    print("MAG X:{0:10.2f}, MAG Y:{1:10.2f}, MAG Z:{2:10.2f} uT".format(mag_x, mag_y, mag_z))
    print("Acceleration: X:%.2f, Y: %.2f, Z: %.2f m/s^2" % (sensor.acceleration))
    print("Gyro X:%.2f, Y: %.2f, Z: %.2f radians/s" % (sensor.gyro))
    print("")
    time.sleep(0.1)