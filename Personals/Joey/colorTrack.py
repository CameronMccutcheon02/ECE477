import cv2
import numpy as np
import time

video = 'airhockey.mp4'
webcam = 0
vid = cv2.VideoCapture(video)

low_red = np.array([0, 150, 150])
high_red = np.array([150, 255, 200])
old_centroid = (0, 0)
velocity = (0, 0)
prediction = (0, 0)

while(True):
    ret, frame = vid.read()
    copy = frame.copy()
    roi = frame
    
    if ret == True:
        
        # Convert frame to HSV spectrum, mask using desired color range
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, low_red, high_red)
        #cv2.imshow("color", mask)
    
        # Make all non-white colors black, find contours
        _, mask = cv2.threshold(mask, 254, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Find largest contour, draw bounding box, find center
        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)
        cv2.rectangle(roi, (x, y), (x+w, y+h), (0, 255, 0), 3)
        moments = cv2.moments(largest_contour)
        centroid = (int(moments["m10"] / moments["m00"]), int(moments["m01"] / moments["m00"]))

        print(f'Velocity: {velocity}')
        print(f'Prediction: {prediction}')
        print(f'Actual: {centroid}\n\n')
            
        # Compute velocity and prediction based off of previous frame
        velocity = (centroid[0] - old_centroid[0], centroid[1] - old_centroid[1])
        prediction = (centroid[0] + velocity[0], centroid[1] + velocity[1]) 
        old_centroid = centroid



    else:
        vid = cv2.VideoCapture('airhockey.mp4')
        continue

    cv2.imshow('Frame', frame)
    time.sleep(1)

    if cv2.waitKey(1) == 27:
        break

vid.release()
cv2.destroyAllWindows()
