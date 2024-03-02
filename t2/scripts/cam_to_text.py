import cv2
from cvzone.HandTrackingModule import HandDetector
import numpy as np
from cvzone.ClassificationModule import Classifier
import math
import time
import os
import rospy
from std_msgs.msg import String

# pip install ( cvzone - mediapipe - tensorflow)

def publisher_node():
    rospy.init_node('cam_to_text', anonymous=True)
    pub = rospy.Publisher('cam_to_speech', String, queue_size=1)
    rate = rospy.Rate(10)  # 10 Hz

    cap = cv2.VideoCapture(0)
    detector = HandDetector(maxHands=1)
    classifier = Classifier("catkin_ws/src/t2/scripts/keras_model.h5","catkin_ws/src/t2/scripts/labels.txt")
    offset = 20
    label = ["alf", "bh", "th", "thaa", "geem", "hah", "khaa", "dal", "zal", "rh", "zen", "seen", "shhen", "sad", "dad", "tah", "zad", "3en", "8en", "fh", "kf", "kaaf", "lam", "mem", "non", "hh", "wow", "yh"]
    index = 0

    while not rospy.is_shutdown():
        success, img = cap.read()
        imgOut = img.copy()
        hands, img = detector.findHands(img)
        
        if hands:
            hand = hands[0]
            x, y, w, h = hand['bbox']

            # Add checks for valid bounding box coordinates
            if x >= 0 and y >= 0 and w > 0 and h > 0 and x + w < img.shape[1] and y + h < img.shape[0]:
                imgwhite = np.ones((300, 300, 3), np.uint8) * 255
                
                imgcrop = img[y-offset:y + h+offset, x-offset: x + w+offset]

                # Add a check for valid cropped image
                if isinstance(imgcrop, np.ndarray) and imgcrop.size != 0 and imgcrop.shape[0] > 0 and imgcrop.shape[1] > 0:
                    aspectRatio = h / w
                    if aspectRatio > 1:
                        k = 300 / h
                        wcal = math.ceil(k * w)
                        imgResize = cv2.resize(imgcrop, (wcal, 300))
                        wgap = math.ceil((300 - wcal) / 2)
                        imgwhite[:, wgap:wcal+wgap] = imgResize
                    else:
                        k = 300 / w
                        hcal = math.ceil(k * h)
                        imgResize = cv2.resize(imgcrop, (300, hcal))
                        hgap = math.ceil((300 - hcal) / 2)
                        imgwhite[hgap:hcal+hgap, :] = imgResize
                    
                    prediction, index = classifier.getPrediction(imgwhite)
                    cv2.putText(imgOut, label[index], (x, y-20), cv2.FONT_HERSHEY_COMPLEX, 2, (255, 0, 255), 2)

                    cv2.imshow("imagecrop", imgcrop)  # Display cropped image for debugging
                    cv2.imshow("imagewhite", imgwhite)

        cv2.imshow("image", imgOut)
        key = cv2.waitKey(1)

        data_to_publish = label[index] if hands else "No hand detected"  # Publish only when hand is detected
        rospy.loginfo("Publishing: {}".format(data_to_publish))
        pub.publish(data_to_publish)
        rate.sleep()

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        publisher_node()
    except rospy.ROSInterruptException:
        pass
