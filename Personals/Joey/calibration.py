import cv2
import numpy as np


# shape is (row, column, (bgr))
def findRange(frame):
    difference = 50
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    shape = frame.shape
    color = frame[int(shape[0]/2), int(shape[1]/2)]
    low = np.array([max((color[0]-difference)/2.55, 0), 50, 50])
    high = np.array([min((color[0]+difference)/2.55, 255), 255, 255])
    
    return low, high
    
    
    




