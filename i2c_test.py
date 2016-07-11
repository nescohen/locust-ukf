#!/usr/bin/python
#Success

import smbus

bus = smbus.SMBus(1)

DEVICE_ADDRESS = 0x19

val = bus.read_byte_data(DEVICE_ADDRESS, 0x20)
print(bin(val))
