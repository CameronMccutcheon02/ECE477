# import the opencv library
import cv2
import serial

# import tracking library
from tracker import *

# Create tracker object
tracker = EuclideanDistTracker()

# define a video capture object
cap = cv2.VideoCapture("C:/Users/halus/Documents/ECE477/airhockey_slow.mp4")  # predefined file
ser = serial.Serial("COM6") #, 115200, timeout=None, xonxoff=False, rtscts=False, dsrdtr=False)

# object detection for stable camera
object_detector = cv2.createBackgroundSubtractorMOG2(history=70, varThreshold=50) # the history and varThreshold impact precision # extracts moving objects from stable camera

while True:
    ret, frame = cap.read()
    
    # extract a region of interest
    roi = frame[100:620, 200:1000]
    
    # object detection
    mask = object_detector.apply(frame) # apply the mask to each frame
    _, mask = cv2.threshold(mask, 254, 255, cv2.THRESH_BINARY)  # white color is 255, everything else is not white
    countours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)   # extract different boundaries of the white objects
    for contour in countours:
        # isolate area to remove unwanted elements
        area = cv2.contourArea(contour)
        if area > 4000 and area < 5000:
            #cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2) # trace the countours in green line of 2 thickness
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2) # draw bounding 
            print(x, y)
            ser.write()
    #cv2.imshow("roi", roi)
    cv2.imshow("Mask", mask)    # show the mask, objects are in white
    #cv2.imshow("Frame", frame)

    key = cv2.waitKey(30)
    if(key==27):
        break

cap.release()
cv2.destroyAllWindows()
