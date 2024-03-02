#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Int8, Bool

def publisher_node():
    rospy.init_node('keyboard_node',anonymous=True)
    pub = rospy.Publisher('chatter1',Int8, queue_size=10)
    rate = rospy.Rate(1)  #1 Hz

    while not rospy.is_shutdown():
        data_to_publish = Int8(input("degree: "))  # Change this value as needed
        rospy.loginfo("Publishing: {}".format(data_to_publish))
        pub.publish(data_to_publish)
        rate.sleep()


if __name__ == '__main__':
    try:
        publisher_node()
    except rospy.ROSInterruptException:
        pass
    