from ultralytics import YOLO
import cv2

cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)

model = YOLO("/home/kareem/catkin_ws/src/t2/scripts/model1.pt")
label = ["ع", "ال", "أ", "ب", "د", "ظ", "ض", "ف", "ق", "غ", "ه", "ح", "ج", "ك", "خ", "أحبك", "ل", "م", "ن", "ر", "ص", "س", "ش", "ت", "ط", "ث", "ذ", "ة", "و", "يا", "ي", "ز"]

while True:
    sucess, img = cap.read()
    results = model.predict(img, show=True)

    for r in results:
        boxes = r.boxes
        for box in boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 255), 3)
            class_label = label[int(box.cls)]
            print("Detected class label:", class_label)
            print("Detected class label number:", int(box.cls))

    cv2.imshow("Image", img)
    cv2.waitKey(1)
