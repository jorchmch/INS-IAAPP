import funciones
import time
import mahony
import board
import array as arr

import sdioio
import storage
import busio

import re
import struct

from adafruit_lsm6ds.lsm6dsox import LSM6DSOX as LSM6DS
from adafruit_lsm6ds import Rate, AccelRange, GyroRange

from adafruit_lis3mdl import LIS3MDL, PerformanceMode, Range

from adafruit_lis3mdl import Rate as RateLis

sdcard = sdioio.SDCard(
    clock=board.SDIO_CLOCK,
    command=board.SDIO_COMMAND,
    data=board.SDIO_DATA,
    frequency=25000000)

vfs = storage.VfsFat(sdcard)

storage.mount(vfs, "/sd")

# import digitalio
radtodeg = 57.2957795
# ---------------------------------------------- #
# Inicializacion de UART
# ---------------------------------------------- #
uart = busio.UART(board.TX, board.RX, baudrate=38400, timeout=0.05)

# ---------------------------------------------- #
# Inicializacion de I2C
# ---------------------------------------------- #
(gyr_acc, mag, i2cControlador) = funciones.Inicializacion()

# ---------------------------------------------- #
# Inicializacion Variables Kalman FIlter
# ---------------------------------------------- #
activ_gX = True
xesti_gX = 0
pesti_gX = 1

activ_gY = True
xesti_gY = 0
pesti_gY = 1

activ_gZ = True
xesti_gZ = 0
pesti_gZ = 1

activ_yaw = True
xesti_yaw = 0
pesti_yaw = 1

activ_roll = True
xesti_roll = 0
pesti_roll = 1

activ_pitch = True
xesti_pitch = 0
pesti_pitch = 1

# ---------------------------------------------- #
# calibracion magnetometro
# ---------------------------------------------- #
'''
Xmin, Xmax, Ymin, Ymax, Zmin, Zmax = funciones.calib_mag(sensor, magne)
print(Xmin, Xmax, Ymin, Ymax, Zmin, Zmax)

print(Xmin, Xmax, Ymin, Ymax, Zmin, Zmax)
# Resultados previos calculados
'''
Xmin, Xmax, Ymin, Ymax, Zmin, Zmax = (
    5.89009,
    42.8676,
    -10.2455,
    30.6197,
    -0.467699,
    32.8997,
)

# ---------------------------------------------- #
# create the ahrs_filter
# ---------------------------------------------- #
ahrs_filter = mahony.Mahony(25, 5, 50)

# ---------------------------------------------- #
# variables
# ---------------------------------------------- #
result = bytearray(12)
timestamp = time.monotonic_ns()
print("inicio")
while True:

    (ace_x,ace_y,ace_z,gyr_x,gyr_y,gyr_z,mag_x,mag_y,mag_z) = funciones.datosIMU(gyr_acc, mag, i2cControlador)
    # adjust for magnetic calibration - hardiron only
    # calibration varies per device and physical location
    mag_x = funciones.map_range(mag_x, Xmin, Xmax, -1, 1)
    mag_y = funciones.map_range(mag_y, Ymin, Ymax, -1, 1)
    mag_z = funciones.map_range(mag_z, Zmin, Zmax, -1, 1)

    # Kalman Filter
    xesti_gX, pesti_gX, activ_gX= funciones.kalmanFilter(gyr_x,1,10,xesti_gX, pesti_gX,activ_gX)
    xesti_gY, pesti_gY, activ_gY= funciones.kalmanFilter(gyr_y,1,10,xesti_gY, pesti_gY,activ_gY)
    xesti_gZ, pesti_gZ, activ_gZ= funciones.kalmanFilter(gyr_z,1,10,xesti_gZ, pesti_gZ,activ_gZ)

    # adjust for my gyro calibration values
    # calibration varies per device and physical location
    xesti_gX += 0  # calibration
    xesti_gY += 0  # calibration
    xesti_gZ += 0  # calibration


    # update the ahrs_filter with the values

    ahrs_filter.update(
        xesti_gX, -xesti_gY, -xesti_gZ, ace_x, -ace_y, -ace_z, mag_x, -mag_y, -mag_z
    )

    yaw = ahrs_filter.yaw * radtodeg
    pitch = ahrs_filter.pitch * radtodeg
    roll = ahrs_filter.roll * radtodeg

    xesti_yaw, pesti_yaw, activ_yaw= funciones.kalmanFilter(yaw,0.1,1,xesti_yaw, pesti_yaw,activ_yaw)
    xesti_pitch, pesti_pitch, activ_pitch= funciones.kalmanFilter(pitch,0.1,1,xesti_pitch, pesti_pitch,activ_pitch)
    xesti_roll, pesti_roll, activ_roll= funciones.kalmanFilter(roll,0.1,1,xesti_roll, pesti_roll,activ_roll)
    # print((xesti_yaw,xesti_pitch,xesti_roll))


    try:
        while not i2cControlador.try_lock():
            pass

        i2cControlador.readfrom_into(0x08, result)

        i2cControlador.unlock()
        #print(result)
        lat = result[:4]
        lon = result[4:8]
        alt = result[8:]
        lat = struct.unpack('<f', lat)[0]
        lon = struct.unpack('<f', lon)[0]
        alt = struct.unpack('<f', alt)[0]

        with open("/sd/log.txt", "a") as f:
            f.write(str(xesti_yaw)+","+str(xesti_pitch)+","+str(xesti_roll)+","+str(lat)+","+str(lon)+","+str(alt)+"\r\n")

        mensaje = funciones.enviar(xesti_yaw, xesti_pitch , xesti_roll ,lat,lon,alt)
        uart.write(bytes(mensaje+"\r\n", "utf-8"))

        print((xesti_yaw,xesti_pitch,xesti_roll,lat,lon,alt))


    except OSError:
        pass

    except UnicodeError:
        pass

    except IndexError:
        pass

    except AttributeError:
        pass

    time.sleep(0.1)
