#!/usr/bin/python
#Success

import smbus

bus = smbus.SMBus(1)

DEVICE_ADDRESS = 0x69

bus.write_byte_data(DEVICE_ADDRESS, 0x20, 0x0F)

while 1:
	val = bus.read_byte_data(DEVICE_ADDRESS, 0x29)
	print(bin(val))
