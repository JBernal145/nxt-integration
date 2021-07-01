#!/usr/bin/env python
# -*- coding: utf-8 -*-
# =============================================================================
# Created By  : Javier Bernal
# Created Date: 2021-04-15
# =============================================================================
"""This module check the communication latency between PC and NXT """
# =============================================================================
# Imports
# =============================================================================
import time
import nxt.locator
from nxt.sensor import *

print '########################################'
print '#                                      #'
print '#             LATENCY TEST             #'
print '#         Integration NXT-ROS          #'
print '########################################'

print 'Connecting with the NXT...'
brick = nxt.locator.find_one_brick(debug=False, method=nxt.Method(usb=False, bluetooth=True))
print 'Connected!'
print '---------------------------------------\n'

#Touch sensor latency test
# touch = Touch(b, PORT_2)
# start = time.time()
# for i in range(100):
#     touch.get_sample()
# stop = time.time()
# print 'touch latency: %s ms' % (1000 * (stop - start) / 100.0)

#Color sensor latency test
color = Color20(brick, PORT_1)
start = time.time()
for i in range(100):
    color.get_color()
stop = time.time()
print 'Color latency: %s ms' % (1000 * (stop - start) / 100.0)

#Ultrasonic sensor latency test
ultrasonic = Ultrasonic(brick, PORT_2)
start = time.time()
for i in range(100):
    ultrasonic.get_distance()
stop = time.time()
print 'Ultrasonic latency: %s ms' % (1000 * (stop - start) / 100.0)

#Gyro sensor latency test
gyro = HTGyro(brick, PORT_3)
start = time.time()
for i in range(100):
    gyro.get_sample()
stop = time.time()
print 'Gyro latency: %s ms' % (1000 * (stop - start) / 100.0)

#Acc sensor latency test
acc = HTAccelerometer(brick, PORT_4)
start = time.time()
for i in range(100):
    acc.get_sample()
stop = time.time()
print 'Acc latency: %s ms' % (1000 * (stop - start) / 100.0)
