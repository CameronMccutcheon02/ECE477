import cv2
import numpy as np
import time
from calibration import findRange
from trackFunctions import *



if __name__ == '__main__':

    #initialize video input and color ranges
    video = 'green2.mp4'
    camera = 0
    vid = cv2.VideoCapture(video)
    malletPic = cv2.imread('falseMallet2.jpg')

    low_puck = np.array([40, 40, 100])
    high_puck = np.array([80, 80, 255])
    low_mallet, high_mallet = findRange(malletPic)

    #initialize locations
    puck_location = (0, 0)
    old_puck_location = (0, 0)
    mallet_location = (0, 0)
    old_mallet_location = (0, 0)

    while(True):

        ret, frame = vid.read()
        if ret == False:
            break

        frame = cv2.resize(frame, (960, 540))
        
        
        puck_location = findObject(frame, low_puck, high_puck)
        if puck_location == None: puck_location = old_puck_location

        mallet_location = findObject(frame, low_mallet, high_mallet)
        if mallet_location == None: mallet_location = old_mallet_location

        # Compute velocity and prediction based off of previous frame
        velocity = (puck_location[0] - old_puck_location[0], puck_location[1] - old_puck_location[1])
        prediction = (puck_location[0] + velocity[0], puck_location[1] + velocity[1]) 

        old_puck_location = puck_location
        old_mallet_location = mallet_location

        slope = int(velocity[1]/(velocity[0]+1e-5))
        puckLine = Line(slope, puck_location) #defines line on which puck is traveling

        findBounce(frame, velocity, puck_location, puckLine, 3)
            
        # cv2.circle(frame, (150, 540), 15, (0, 255, 0), -1)
        # cv2.circle(frame, (150, 0), 15, (0, 255, 0), -1)
        # cv2.circle(frame, (770, 540), 15, (0, 255, 0), -1)
        # cv2.circle(frame, (770, 0), 15, (0, 255, 0), -1)
                
        cv2.imshow('Frame', frame)
        time.sleep(0.04)

        if cv2.waitKey(1) == 27:
            break

    vid.release()
    cv2.destroyAllWindows()