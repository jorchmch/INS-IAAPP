import busio
import board
import time
import struct

i2c = busio.I2C(board.SCL, board.SDA)

while not i2c.try_lock():
    pass

timestamp = 0
count = 0
while(True):

    if (time.monotonic_ns() - timestamp) > 100000000:  # cada 0.1 S 10Hz


        result = bytearray(12)
        i2c.readfrom_into(0x8, result)
        #print(result)

        lat = result[:4]
        lon = result[4:8]
        alt = result[8:]
        lat = struct.unpack('<f', lat)[0]
        lon = struct.unpack('<f', lon)[0]
        alt = struct.unpack('<f', alt)[0]

        print("%.8f" % lat,"%.8f" % lon,"%.3f" % alt,count)

        count = count + 1

        timestamp = time.monotonic_ns()
