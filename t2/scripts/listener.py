import rospy
from std_msgs.msg import String, Int16, Bool

def my_callback(my_string):
    rospy.loginfo("%s", my_string.data)
    

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter1", Int16, my_callback) #("same topic for puplisher",data type, function you need to excute)
    rospy.spin()


if __name__== '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass