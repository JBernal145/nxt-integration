#!/usr/bin/env python

import time
import nxt.locator
from nxt.sensor import *

print '###################################'
print '#  CONNECTIVITY AND SENSORS TEST  #'
print '###################################'

print 'Connecting with the NXT...'
brick = nxt.locator.find_one_brick(debug=False, method=nxt.Method(usb=False, bluetooth=True))
print 'Connected!'
print '-----------------------------------\n'

color = Color20(brick, PORT_1)
u = Ultrasonic(brick, PORT_2)
gyro = HTGyro(brick, PORT_3)
acc = HTAccelerometer(brick, PORT_4)

while True:
    print 'Color: \t', color.get_color()
    print 'Ultrasonic: \t', u.get_sample()
    print 'Gyro without calibrate: \t', gyro.get_rotation_speed()
    print '...calibrating...'
    gyro.calibrate()
    print 'Gyro calibrated: \t', gyro.get_rotation_speed()

    acceleration = acc.get_acceleration()
    print 'Accelerometer X:', acceleration.x
    print 'Accelerometer Y:', acceleration.y
    print 'Accelerometer Z:', acceleration.z

    print '------------------------------------'
    print ''

    time.sleep(5)
