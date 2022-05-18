"""CircuitPython Essentials UART Serial example"""
import board
import busio
import digitalio

import sdioio
import storage


uart = busio.UART(board.TX, board.RX, baudrate=9600)

led = digitalio.DigitalInOut(board.LED)
led.direction = digitalio.Direction.OUTPUT

sdcard = sdioio.SDCard(
    clock=board.SDIO_CLOCK,
    command=board.SDIO_COMMAND,
    data=board.SDIO_DATA,
    frequency=25000000)

vfs = storage.VfsFat(sdcard)

storage.mount(vfs, "/sd")



while True:
    data = uart.readline()

    if data is None:
        led.value = False
    else:
        dataNum = int(data)
        print(dataNum)
        led.value = True

    #with open("/sd/test11.txt", "a") as f:
        #f.write("Numero: " + str(dataNum) + "\r\n")