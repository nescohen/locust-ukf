#!/usr/bin/python
#Success

import smbus

bus = smbus.SMBus(1)

DEVICE_ADDRESS = 0x69

val = bus.read_byte_data(DEVICE_ADDRESS, 0x2E)
print(bin(val))
