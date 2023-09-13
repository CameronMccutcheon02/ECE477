# import the opencv library
import cv2
import numpy as np

# define a video capture object
cap = cv2.VideoCapture("C:/Users/halus/Documents/ECE477/airhockey.mp4")

# object detection for stable camera
object_detector = cv2.createBackgroundSubtractorMOG2(history=70, varThreshold=50) # the history and varThreshold impact precision # extracts moving objects from stable camera

while True:
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray_blur = cv2.blur(gray, (3,3))
    detected_circles = cv2.HoughCircles(gray_blur, cv2.HOUGH_GRADIENT, 1, 20, param1=60, param2=30, minRadius=1, maxRadius=40)

    mask = object_detector.apply(frame) # apply the mask to each frame
    _, mask = cv2.threshold(mask, 254, 255, cv2.THRESH_BINARY)  # white color is 255, everything else is not white
    countours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)   # extract different boundaries of the white objects

    for contour in countours:
        # isolate area to remove unwanted elements
        area = cv2.contourArea(contour)
        if area > 4000 and area < 5000 and detected_circles is not None:
            #cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2) # trace the countours in green line of 2 thickness
            x, y, w, h = cv2.boundingRect(contour)
            detected_circles = np.uint16(np.around(detected_circles))
            for circle in detected_circles[0, :]:
                a, b, r = circle[0], circle[1], circle[2]
                if (r > 29):
                    cv2.circle(frame, (a, b), r, (0, 255, 0), 2)
                    cv2.imshow("detected circle", frame)
                
    key = cv2.waitKey(30)
    if(key==27):
        break

cap.release()
cv2.destroyAllWindows()