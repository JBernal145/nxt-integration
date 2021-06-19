#!/usr/bin/env python

import time
import nxt.locator
from nxt.sensor import *

b = nxt.locator.find_one_brick()

#Touch sensor latency test
# touch = Touch(b, PORT_2)
# start = time.time()
# for i in range(100):
#     touch.get_sample()
# stop = time.time()
# print 'touch latency: %s ms' % (1000 * (stop - start) / 100.0)

#Color sensor latency test
color = Color20(b, PORT_1)
start = time.time()
for i in range(100):
    color.get_color()
stop = time.time()
print 'Color latency: %s ms' % (1000 * (stop - start) / 100.0)

#Ultrasonic sensor latency test
ultrasonic = Ultrasonic(b, PORT_2)
start = time.time()
for i in range(100):
    ultrasonic.get_distance()
stop = time.time()
print 'Ultrasonic latency: %s ms' % (1000 * (stop - start) / 100.0)

#Gyro sensor latency test
gyro = HTGyro(b, PORT_3)
start = time.time()
for i in range(100):
    gyro.get_sample()
stop = time.time()
print 'Gyro latency: %s ms' % (1000 * (stop - start) / 100.0)

#Acc sensor latency test
acc = HTAccelerometer(b, PORT_4)
start = time.time()
for i in range(100):
    acc.get_sample()
stop = time.time()
print 'Acc latency: %s ms' % (1000 * (stop - start) / 100.0)
