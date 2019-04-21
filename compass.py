#!/usr/bin/python3

from IMU import IMU
import time

print('start')

i = IMU()
#c.begin()
while True:
    a, m = i.get()
    print(i.getOrientation(a, m))
    time.sleep(0.05)
