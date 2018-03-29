#!/usr/bin/env python

from pprint import pprint
# import struct
import pyusbxp
from datetime import datetime
my_USB_thingy = pyusbxp.Usbxp()
pprint(my_USB_thingy)
print("Open:", my_USB_thingy.open())
size, version = my_USB_thingy.read(2, 'B'*2)
print("   + size:", size, "version:", version)
my_USB_thingy.read(size-2)
for i in range(1, 101):
    print("{time:%H:%M:%S.%f} Read {i}: size = {rd[0]}; version = {rd[1]}; state/command = {rd[2]};"
          " measurement = {rd[3]}; data age = {rd[4]}".format(time=datetime.now(),
                                                              i=i, rd=my_USB_thingy.read(size, '>BB5siB')))
pprint(my_USB_thingy)
print("Close:", my_USB_thingy.close())

del my_USB_thingy
