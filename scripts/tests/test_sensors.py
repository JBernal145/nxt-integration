#!/usr/bin/env python
# -*- coding: utf-8 -*-
# =============================================================================
# Created By  : Javier Bernal
# Created Date: 2021-04-15
# =============================================================================
"""The module has been build for test the communication and sensors read"""
# =============================================================================
# Imports
# =============================================================================

import time
import nxt.locator
from nxt.sensor import *

print '#####################################'
print '#   CONNECTIVITY AND SENSORS TEST   #'
print '#####################################'

print 'Connecting with the NXT...'
brick = nxt.locator.find_one_brick(debug=False, method=nxt.Method(usb=False, bluetooth=True))
print 'Connected!'
print '-----------------------------------\n'

# color = Color20(brick, PORT_1)
u = Ultrasonic(brick, PORT_2)
gyro = HTGyro(brick, PORT_3)
# acc = HTAccelerometer(brick, PORT_4)

touch_left = Touch(brick, PORT_1)
touch_right = Touch(brick, PORT_4)

while True:
    # print 'Color: \t\t\t\t', color.get_color()
    print 'Touch left: \t\t', touch_left.is_pressed()
    print 'Touch right: \t\t', touch_right.is_pressed()
    print 'Ultrasonic: \t\t\t', u.get_sample()
    print 'Gyro without calibrate: \t', gyro.get_rotation_speed()
    print '...calibrating...'
    gyro.calibrate()
    print 'Gyro calibrated: \t\t', gyro.get_rotation_speed()

    # acceleration = acc.get_acceleration()
    # print 'Accelerometer X:\t\t', acceleration.x
    # print 'Accelerometer Y:\t\t', acceleration.y
    # print 'Accelerometer Z:\t\t', acceleration.z

    print '------------------------------------'
    print ''

    time.sleep(5)
