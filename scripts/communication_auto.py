#!/usr/bin/env python
# -*- coding: utf-8 -*-
# =============================================================================
# Created By  : Javier Bernal
# Created Date: 2021-04-15
# =============================================================================
"""The module has been build for the connection with NXT and move commands"""
# =============================================================================
# Imports
# =============================================================================

# ROS imports
import rospy
from nxt_integration.msg import Move, Distance, Color, Rotation, Acceleration

# Python and NXT
import nxt.locator
from nxt.sensor import *
from nxt.motor import *
import sys
# =============================================================================

# Connection
brick = nxt.locator.find_one_brick(debug=False, method=nxt.Method(usb=False, bluetooth=True))
m_left = Motor(brick, PORT_B)
m_right = Motor(brick, PORT_A)

# Synchronized motors
both = nxt.SynchronizedMotors(m_left, m_right, 0)
rightboth = nxt.SynchronizedMotors(m_left, m_right, 100)
leftboth = nxt.SynchronizedMotors(m_right, m_left, 100)

# Sensors
color = Color20(brick, PORT_1)
u = Ultrasonic(brick, PORT_2)
gyro = HTGyro(brick, PORT_3)
acc = HTAccelerometer(brick, PORT_4)

# Calibrate Gyro Sensor
gyro.calibrate()


# Callback function for NXT move
def movement(data):
    if data.dir == 'FORWARD':
        both.run(85)  # Power at 85%

    elif data.dir == 'LEFT':
        leftboth.turn(100, 35)  # 90 degrees

    elif data.dir == 'BACK':
        both.run(-85)  # Power at 85%

    elif data.dir == 'RIGHT':
        rightboth.turn(100, 35)

    elif data.dir == 'STOP':
        rightboth.brake()
        leftboth.brake()
        both.brake()

def stop_nxt():
    rightboth.brake()
    leftboth.brake()
    both.brake()

# Publish the data read by the sensor on each topic
def publishments():
    print 'Connected'
    pub_color = rospy.Publisher('color_topic', Color, queue_size=10)
    # pub_touch = rospy.Publisher('touch_topic', TouchSensor, queue_size=10)
    pub_distance = rospy.Publisher('distance_topic', Distance, queue_size=10)
    pub_gyro = rospy.Publisher('gyro_topic', Rotation, queue_size=10)
    pub_acc = rospy.Publisher('acc_topic', Acceleration, queue_size=10)

    rospy.init_node('communication', anonymous=True, disable_signals=True)
    rate = rospy.Rate(10)  # 10hz

    # Loop until stop execution
    while not rospy.is_shutdown():
        msg_color = Color()
        msg_color.header.stamp = rospy.Time.now()
        msg_color.header.frame_id = 'Color_Sensor'
        msg_color.color = color.get_color()
        pub_color.publish(msg_color)

        msg_dist = Distance()
        msg_dist.header.stamp = rospy.Time.now()
        msg_dist.header.frame_id = 'Ultrasonic_Sensor'
        msg_dist.cms = u.get_distance()
        pub_distance.publish(msg_dist)

        msg_gyro = Rotation()
        msg_gyro.header.stamp = rospy.Time.now()
        msg_gyro.header.frame_id = 'Gyro_Sensor'
        msg_gyro.degrees_sec = gyro.get_rotation_speed()
        pub_gyro.publish(msg_gyro)

        msg_acc = Acceleration()
        acceleration = acc.get_acceleration()
        msg_acc.header.stamp = rospy.Time.now()
        msg_acc.header.frame_id = 'Accel_Sensor'
        msg_acc.x = acceleration.x
        msg_acc.y = acceleration.y
        msg_acc.z = acceleration.z
        pub_acc.publish(msg_acc)

        # msg_touch = TouchSensor()
        # msg_touch.header.stamp = rospy.Time.now()
        # msg_touch.header.frame_id = 'Touch_Sensor'
        # msg_touch.left = str(touch_left.is_pressed())
        # msg_touch.right = str(touch_right.is_pressed())
        # pub_touch.publish(msg_touch)

        rate.sleep()


if __name__ == '__main__':
    try:
        # Subscribe to the topic and receive keyboard commands
        rospy.Subscriber('movement_topic', Move, movement)

        # Call for publish in each topic
        publishments()
    except KeyboardInterrupt:
        rightboth.brake()
        leftboth.brake()
        both.brake()
        sys.exit(0)
