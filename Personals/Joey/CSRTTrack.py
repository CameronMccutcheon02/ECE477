import cv2
import numpy as np
import time


cap = cv2.VideoCapture('airhockey.mp4')  

# Define the HSV color range for your target object
lower_color = np.array([0, 150, 150])  # Lower HSV range for your target color
upper_color = np.array([150, 255, 200])  # Upper HSV range for your target color

# Initialize the tracker (CSRT)
tracker = cv2.TrackerCSRT_create()

while True:

    ret, frame = cap.read()

    if not ret:
        break

    # Convert the frame to HSV color space
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create a binary mask to isolate the target color
    mask = cv2.inRange(hsv_frame, lower_color, upper_color)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Initialize object centroid coordinates
    object_centroid = None

    if len(contours) > 0:
        # Find the largest contour (assumed to be the object)
        largest_contour = max(contours, key=cv2.contourArea)

        # Calculate the centroid of the largest contour
        moments = cv2.moments(largest_contour)
        object_centroid = (int(moments["m10"] / moments["m00"]), int(moments["m01"] / moments["m00"]))
        print(object_centroid)

        # Draw a bounding box around the object
        x, y, w, h = cv2.boundingRect(largest_contour)
        

    # If an object is detected, initialize or update the tracker
    if object_centroid:
        if not tracker.init(frame, (x, y, w, h)):
            tracker.update(frame)

        # Get the updated tracker position
        _, new_bbox = tracker.update(frame)
        new_x, new_y, new_w, new_h = [int(i) for i in new_bbox]
        cv2.rectangle(frame, (new_x, new_y), (new_x + new_w, new_y + new_h), (0, 255, 0), 2)

    # Display the frame with object detection and tracking
    cv2.imshow("Object Tracking", frame)

    # Exit the loop if esc is pressed
    if cv2.waitKey(1) == 27:
        break

    #time.sleep(1)

# Release the video capture and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
