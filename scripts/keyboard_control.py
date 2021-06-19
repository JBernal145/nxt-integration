#!/usr/bin/env python
# -*- coding: utf-8 -*-
# =============================================================================
# Created By  : Javier Bernal
# Created Date: 2021-04-15
# =============================================================================
"""This module captures keyboard actions and sends them for communication with
    the NXT Robot"""
# =============================================================================
# Imports
# =============================================================================

# ROS imports
import rospy
from nxt_integration.msg import Move

# Python
import sys
from pynput import keyboard
from termios import tcflush, TCIFLUSH

# Store the previous key pressed
previous_key = ''

# Declare script as publisher
pub_movement = rospy.Publisher('movement_topic', Move, queue_size=10)

# Message creation
msg_move = Move()
msg_move.header.frame_id = 'Keyboard_Movement'


# Function that publish in topic when press compatible keys (w, a ,s, d)
def on_press(key):
    global previous_key

    try:
        if previous_key != key.char:
            if key.char == 'w' or key.char == 'a' or key.char == 's' or key.char == 'd':
                previous_key = key.char

                # Send movement
                msg_move.dir = key.char
                msg_move.header.stamp = rospy.Time.now()
                pub_movement.publish(msg_move)

    except AttributeError:
        print('special key {0} pressed'.format(
            key))
        return True


# Send stop command when the key is released
def on_release(key):
    global previous_key

    # Reset global parameter
    previous_key = ''

    # Send stop all motors
    msg_move.header.stamp = rospy.Time.now()
    msg_move.dir = 'STOP'
    pub_movement.publish(msg_move)

    # Stop execution with Esc, Space or 'q'
    if key == keyboard.Key.esc or key == keyboard.Key.space or key.char == 'q':
        # Stop listener
        tcflush(sys.stdin, TCIFLUSH)
        return False


# Node creation and control for pressed and released key
def remote_control():
    # Create node
    rospy.init_node('keyboard_control', anonymous=True)

    # Loop until stop execution
    while not rospy.is_shutdown():
        # Listen actions in keyboard
        rospy.loginfo('Listening commands')
        with keyboard.Listener(
                on_press=on_press,
                on_release=on_release) as listener:
            try:
                listener.join()
            except AttributeError:
                tcflush(sys.stdin, TCIFLUSH)
                print 'Special Keys is not allowed'

        rospy.loginfo('Aborted!')


if __name__ == '__main__':
    try:
        remote_control()
    except rospy.ROSInterruptException:
        pass
