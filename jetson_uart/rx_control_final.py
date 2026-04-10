#!/usr/bin/python3
import time
import serial
import copy
import keyboard

def CharConvertion(dataRX):
    newList2 = dataRX.decode("utf-8")
    newList = newList2.split(" ")
    newFloatList = [float(n) for n in newList]
    return newFloatList 

print("UART Demonstration Program")
print("NVIDIA Jetson Nano Developer Kit")


serial_port = serial.Serial(
    port="/dev/ttyUSB0",
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
)
# Wait a second to let the port initialize
time.sleep(1)


while True:
    if serial_port.inWaiting() > 0:
        firstChar = serial_port.read(1)
        print(ord(firstChar))
        if ord(firstChar) == 37:
            start = time.time()
            dataRX = serial_port.read(69)
            end = time.time()
            # buffer = copy.copy(data)
            print(dataRX)
            print('\n\r')
            print(end - start)
            print('\n\r')
            serial_port.write(dataRX)
            try:
                ConvertedList = CharConvertion(dataRX)
                print(ConvertedList)
            except Exception:
                pass

    if keyboard.is_pressed('q'):
        serial_port.close()
        serial_port.__del__()

# Optimized string splitting and float list comprehensions to reduce callback latency.
