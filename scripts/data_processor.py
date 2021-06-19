#!/usr/bin/env python
# -*- coding: utf-8 -*-
# =============================================================================
# Created By  : Javier Bernal
# Created Date: 2021-04-15
# =============================================================================
"""The script process the data published in each topic for the NXT sensors"""
# =============================================================================
# Imports
# =============================================================================

# ROS imports
import rospy
import rosbag
from nxt_integration.msg import Rotation, Acceleration, Distance, Color

bag = rosbag.Bag('sensor_data.bag', 'w')

def write_rotation(data):
    bag.write('Rotation_Data', data)

def write_position(data):
    bag.write('Acceleration_Data', data)

def write_distance(data):
    bag.write('Distance_Data', data)

def write_color(data):
    bag.write('Color_Data', data)

def data_processor():
    try:
        # Node initialization
        rospy.init_node('data_processor', anonymous=True)

        # Subscribe to Gyro and Acceleration data (position and rotation)
        rospy.Subscriber('gyro_topic', Rotation, write_rotation)
        rospy.Subscriber('acc_topic', Acceleration, write_position)
        rospy.Subscriber('distance_topic', Distance, write_distance)
        rospy.Subscriber('color_topic', Color, write_color)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    finally:
        bag.close()
        pass

if __name__ == '__main__':
    data_processor()
