#!/usr/bin/python3
import time
import serial
import copy
import keyboard

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


buffer = []
Rx_buffer = []

try:
    # Send a simple header
    # serial_port.write("UART Demonstration Program\r\n".encode())
    # serial_port.write("NVIDIA Jetson Nano Developer Kit\r\n".encode())
    while True:
        if serial_port.inWaiting() > 0:
            reading = serial_port.read(1)
            print(ord(reading))
            if ord(reading) == 37:
                print("percentage read")
        if keyboard.is_pressed('q'):
            serial_port.close()
            serial_port.__del__()
            
            


except KeyboardInterrupt:
    print("Exiting Program")
    serial_port.__del__()

except Exception as exception_error:
    print("Error occurred. Exiting Program")
    print("Error: " + str(exception_error))
    serial_port.__del__()

finally:
    print("ciao")
    serial_port.__del__()
    # serial_port.close()
    pass
