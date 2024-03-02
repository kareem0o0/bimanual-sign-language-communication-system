from gtts import gTTS
from playsound import playsound
import speech_recognition as sr
import rospy
from std_msgs.msg import String
# -*- coding: utf-8 -*-
# Create a speech recognition object
# important note : write this line befor installing pyaudio : sudo apt-get install python3-dev portaudio19-dev

def publisher_node():
    rospy.init_node('speech_to_text', anonymous=True)
    pub = rospy.Publisher('speech_to_arduino', String, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        r = sr.Recognizer()
        t = None 
        try:
    # Use the microphone as the audio source with a timeout of 2 seconds
            with sr.Microphone(None) as src:
                print('Say something, brother')
                # Record audio for 2 seconds
                audio = r.record(src, duration=3)

            # Attempt to recognize speech using Google's speech recognition API
            t = r.recognize_google(audio, language='ar-AR')
            print(t)

            # Append the recognized text to a file named 'text.txt'
            #with open('text.txt', 'a', encoding='utf-8') as f:
            #   f.write(t + '\n')

            # Create a gTTS (Google Text-to-Speech) object and save it as an audio file
            #obj = gTTS(text=t, lang='ar', slow=False)
            #obj.save('text.mp3')

            # Play the generated audio file
            #playsound('text.mp3')

        except sr.UnknownValueError as U:
            # Handle cases where speech cannot be recognized
            print(U)

        except sr.RequestError as R:
            # Handle cases where there is an issue with the recognition service (e.g., no internet connection)
            print(R)
        data_to_publish = t # Change this value as needed
        rospy.loginfo("Publishing: {}".format(data_to_publish))
        pub.publish(t)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher_node()
    except rospy.ROSInterruptException:
        pass
