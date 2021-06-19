from __future__ import division
import nxt.locator
from nxt.sensor import *
import sys
import time
import tty, termios
from nxt.motor import *

b = nxt.locator.find_one_brick(debug=True, method=nxt.Method(usb=False, bluetooth=True))
m_left = Motor(b, PORT_B)
m_right = Motor(b, PORT_A)

both = nxt.SynchronizedMotors(m_left, m_right, 0)
rightboth = nxt.SynchronizedMotors(m_left, m_right, 100)
leftboth = nxt.SynchronizedMotors(m_right, m_left, 100)

# Variable measurements
measurements = 0

# Gyro sensor
gyro = HTGyro(b, PORT_3)
gyro.calibrate()

leftboth.run()
start_measure = time.time()
for i in range(100):
    medida_actual = gyro.get_rotation_speed()
    measurements += medida_actual
    with open('Exported.txt', 'a+') as file:
        cadena = 'Rotation speed: ' + str(medida_actual) + '\n'
        file.write(cadena)

stop_measure = time.time()
rightboth.brake()
leftboth.brake()
both.brake()
print 'Gyro latency: %s ms' % (1000 * (stop_measure - start_measure) / 100.0)
print 'Medida total: ' + str(measurements)


# Calculamos la media
degrees_by_second = measurements / 100

print 'Grados por segundo: ' + str(degrees_by_second)
time.sleep(5)

if degrees_by_second < 0:
    seconds_90 = 360 / (degrees_by_second * (-1))
else:
    seconds_90 = 360 / degrees_by_second

print 'Tiempo para 90 grados: ' + str(seconds_90)
seconds = seconds_90 + 0.3
start_time = time.time()

while True:
    leftboth.run()
    current_time = time.time()
    elapsed_time = current_time - start_time

    if elapsed_time > seconds:
        rightboth.brake()
        leftboth.brake()
        both.brake()
        print("Finished iterating in: " + str(elapsed_time) + " seconds")
        break
