#!/usr/bin/python3
import time
import serial
import keyboard

print("UART Demonstration Program")
print("NVIDIA Jetson Nano Developer Kit")


serial_port = serial.Serial(
    port="/dev/ttyTHS0",
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=0
)
# Wait a second to let the port initialize
time.sleep(1)
data = ''

try:
    # Send a simple header
    serial_port.write("UART Demonstration Program\r\n".encode())
    serial_port.write("NVIDIA Jetson Nano Developer Kit\r\n".encode())
    while True:
        if keyboard.is_pressed('q'):
            print('\n')
            data = input()
        coso = serial_port.read()
        if (data != ''):
            serial_port.write(data.encode())
            #if data == "\r".encode():
                # For Windows boxen on the other end
                #serial_port.write("\n".encode())
        data = ''
        print(data)


except KeyboardInterrupt:
    print("Exiting Program")

except Exception as exception_error:
    print("Error occurred. Exiting Program")
    print("Error: " + str(exception_error))

finally:
    serial_port.close()
    pass
