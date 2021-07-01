#!/usr/bin/env python
# -*- coding: utf-8 -*-
# =============================================================================
# Created By  : Javier Bernal
# Created Date: 2021-05-17
# =============================================================================
"""This module allows NXT autonomous navigation, send the movements and decide
when an obstacle is near"""
# =============================================================================
# Imports
# =============================================================================

# ROS imports
import rospy
from nxt_integration.msg import Move, Rotation, Acceleration, Distance

# Python
import sys
import time

# Declare script as publisher
pub_movement = rospy.Publisher('movement_topic', Move, queue_size=10)

# Message creation
msg_move = Move()
msg_move.header.frame_id = 'Autonomous_Movement'

last_move = 'START'

def write_movement(data):
    global last_move
    if data.cms > 25 and last_move != 'FORWARD':
        # Send movement
        msg_move.dir = 'FORWARD'
        msg_move.header.stamp = rospy.Time.now()
        pub_movement.publish(msg_move)
        last_move = msg_move.dir
    elif data.cms <= 25:
        # Check last move, avoid repeat same message
        if last_move != 'STOP' and (last_move == 'FORWARD' or last_move == 'START'):
            msg_move.dir = 'STOP'
            msg_move.header.stamp = rospy.Time.now()
            pub_movement.publish(msg_move)
            last_move = msg_move.dir
        elif last_move != 'BACK' and last_move == 'STOP':
            msg_move.dir = 'BACK'
            msg_move.header.stamp = rospy.Time.now()
            pub_movement.publish(msg_move)
            last_move = msg_move.dir
        elif last_move != 'RIGHT' and last_move == 'BACK':
            msg_move.dir = 'RIGHT'
            msg_move.header.stamp = rospy.Time.now()
            pub_movement.publish(msg_move)
            last_move = msg_move.dir
        elif last_move == 'RIGHT':
            msg_move.dir = 'FORWARD'
            msg_move.header.stamp = rospy.Time.now()
            pub_movement.publish(msg_move)
            last_move = 'START'

def autonomous_control():
    rospy.init_node('autonomous_control', anonymous=True)

    # Subscriptions
    rospy.Subscriber('distance_topic', Distance, write_movement)
    rate = rospy.Rate(2)  # 2hz

    # Loop until stop execution
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        autonomous_control()
    except KeyboardInterrupt:
        msg_move.dir = 'STOP'
        msg_move.header.stamp = rospy.Time.now()
        pub_movement.publish(msg_move)
        pass

