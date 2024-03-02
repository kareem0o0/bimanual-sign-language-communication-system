from gtts import gTTS
from playsound import playsound
import rospy
from std_msgs.msg import String
import os

received_data = None

def my_callback(my_string):
    global received_data

    rospy.loginfo("%s", my_string.data)
    received_data = my_string.data

def process_latest_message():
    global received_data
    if received_data is not None:
        try:
            if os.path.exists('text.mp3'):
                os.remove('text.mp3')

            obj = gTTS(received_data, lang='ar', slow=False)
            obj.save('text.mp3')
            if received_data != "No hand detected" :
             playsound('text.mp3')
        except Exception as e:
            rospy.logerr("Error processing message: %s", e)
    
def listener():
    rospy.init_node('text_to_speech', anonymous=True)
    rospy.Subscriber("cam_to_speech", String, my_callback)
    # Process the most recent message continuously
    rate = rospy.Rate(1)  # Adjust rate as needed
    while not rospy.is_shutdown():
        process_latest_message()
        rate.sleep()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
