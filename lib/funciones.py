import board
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX as LSM6DS
from adafruit_lsm6ds import Rate, AccelRange, GyroRange

from adafruit_lis3mdl import LIS3MDL, PerformanceMode, Range

from adafruit_lis3mdl import Rate as RateLis

import time
import mahony
import board
import array as arr

import sdioio
import storage
import busio

import re

def Inicializacion():
    i2c = board.I2C()  # uses board.SCL and board.SDA
    sensor = LSM6DS(i2c)
    magne = LIS3MDL(i2c)

    sensor.accelerometer_range = AccelRange.RANGE_8G
    sensor.gyro_range = GyroRange.RANGE_2000_DPS

    sensor.accelerometer_data_rate = Rate.RATE_416_HZ
    sensor.gyro_data_rate = Rate.RATE_416_HZ

    magne.data_rate = RateLis.RATE_155_HZ
    magne.range = Range.RANGE_4_GAUSS

    return sensor, magne, i2c

def datosIMU(a, b, i2c):
    '''
    while not i2c.try_lock():
        pass
    '''

    acc_x, acc_y, acc_z = a.acceleration
    gyro_x, gyro_y, gyro_z = a.gyro
    mag_x, mag_y, mag_z = b.magnetic

    #i2c.unlock()

    return acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z

def map_range(x, in_min, in_max, out_min, out_max):
    """
    Maps a number from one range to another.
    :return: Returns value mapped to new range
    :rtype: float
    """
    mapped = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    if out_min <= out_max:
        return max(min(mapped, out_max), out_min)

    return min(max(mapped, out_max), out_min)

def kalmanFilter (i,Q,R,x_hat,P_hat,act):
    A=1
    H=1

    if (act == True):
        x_hat = i
        P_hat = 1
        act = False

    xp = A * x_hat
    Pp = (A*P_hat*(1/A))+Q

    K = Pp * (1/H) * ( 1/(H*Pp*(1/H) + R))
    x_hat= xp + K*(i - H*xp)
    P_hat = Pp - K*H*Pp

    return x_hat,P_hat,act

def enviar(a,b,c,d,e,f):    # para flotantes
	a = "{:.2f}".format(a)
	a = float(a)

	b = "{:.2f}".format(b)
	b = float(b)
	c = "{:.2f}".format(c)
	c = float(c)
	
	d = "{:.7f}".format(d)
	d = float(d)
	
	e = "{:.7f}".format(e)
	e = float(e)
	
	f = "{:.2f}".format(f)
	f = float(f)
	
	a = signoNum(a)
	b = signoNum(b)
	c = signoNum(c)
	d = signoNum(d)
	e = signoNum(e)
	f = signoNum(f)

	g = a + ";" + b + ";" + c + ";" + d + ";" + e + ";" + f + "\r"
	return g

def signoNum(a):

    # agregarle el signo
    if (a>0):
        c = str(a)
        b="+"+ '{:0>6}'.format(c)
        #print(b)
    else:
        c = str(a*-1)
        b = "-"+ '{:0>6}'.format(c)

    return b
