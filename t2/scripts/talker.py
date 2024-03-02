#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16

def publisher_node():
    rospy.init_node('publisher_node', anonymous=True)
    pub = rospy.Publisher('chatter1', Int16, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        data_to_publish = int(input("degree: "))  # Change this value as needed
        rospy.loginfo("Publishing: {}".format(data_to_publish))
        pub.publish(data_to_publish)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher_node()
    except rospy.ROSInterruptException:
        pass
