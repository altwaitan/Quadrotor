#!/usr/bin/env python
# Source: https://github.com/mattzzw/Arduino-mpu6050

import serial
import time
import csv
import sys

PORT = '/dev/tty.usbserial-AL03QWTA'
BAUD = 230400

ser = serial.Serial(PORT, BAUD, timeout=1)

while 1:
    global ax, ay, az
    ax = ay = az = 0.0
    ser.write(".")
    line = ser.readline()
    angles = line.split(", ")
    if len(angles) == 3:
        ax = float(angles[0])
        ay = float(angles[1])
        az = float(angles[2])
        line_done = 1
        print("%.2f , %.2f, %.2f" % (ax, ay, az));
