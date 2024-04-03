#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState

def move_robot():
    rospy.init_node('move_robot_node', anonymous=True)
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = ["joint1", "joint2", "joint3"]  # Replace with your joint names
        joint_state.position = [1.0, 0.5, 0.0]  # Replace with desired joint positions
        pub.publish(joint_state)
        rate.sleep()

if __name__ == '__main__':
    try:
        move_robot()
    except rospy.ROSInterruptException:
        pass
