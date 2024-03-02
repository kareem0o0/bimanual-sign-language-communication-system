from gtts import gTTS
from playsound import playsound
import speech_recognition as sr
import rospy
from std_msgs.msg import String


received_data = None

def my_callback(my_string):
    global received_data 
    rospy.loginfo("%s", my_string.data)
    received_data = my_string.data

def listener():
    global received_data
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter1", String, my_callback) 

    # Wait until the first message is received
    while received_data is None and not rospy.is_shutdown():
        rospy.sleep(0.1)

    if received_data is not None:
        obj = gTTS(received_data, lang='ar', slow=False)
        obj.save('text.mp3')
        playsound('text.mp3')

    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
