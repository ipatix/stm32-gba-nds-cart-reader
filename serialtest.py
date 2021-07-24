#!/usr/bin/env python3

import serial
import sys

ser = serial.Serial(port='/dev/ttyACM0', baudrate='115200')

if not ser.isOpen():
    print("failed opening serial device")
    sys.exit(1)

# NDS seek(0) command
request = bytearray([0x57, 0x80, 0xB0, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00])

# send command
ser.write(request)
ser.flush()

# receiver reply header
reply = ser.read(8)
reply_len = reply[6] | (reply[7] << 8)
reply_data = ser.read(reply_len)

# print reply
print("reply header:")
print(reply)
print("reply data:")
print(reply_data)

# NDS read(4096) commad
request = bytearray([0x57, 0x80, 0xB1, 0x00, 0x01, 0x00, 0x02, 0x00, 0x00, 0x02])

# send command
ser.write(request)
ser.flush()

# receiver reply header
reply = ser.read(8)
reply_len = reply[6] | (reply[7] << 8)
reply_data = ser.read(reply_len)

# print reply
print("reply header:")
print(reply)
print("reply data:")

for b in reply_data:
    print("{0:02x} ".format(b), end='')
print()
#print(reply_data)
