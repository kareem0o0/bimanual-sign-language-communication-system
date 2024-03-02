#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String, Int8

def speech_to_move_callback(data):
    global pub_move
    if data.data == "يمين":
        move_msg = 10
    elif data.data == "شمال":
        move_msg = -10
    else:
        return  # Do not publish anything if the message is not "يمين" or "شمال"
    
    rospy.loginfo("Publishing to move: {}".format(move_msg))
    pub_move.publish(move_msg)

def speech_to_move():
    global pub_move
    rospy.init_node('speech_to_arduino', anonymous=True)
    rospy.Subscriber('speech_to_arduino', String, speech_to_move_callback)
    pub_move = rospy.Publisher('move', Int8, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    speech_to_move()
