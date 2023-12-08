import cv2
import numpy as np
from calibration import findRange
from trackFunctions import *
import serial

if __name__ == '__main__':

    #intialize serial instance for uart communication
    ser = serial.Serial(port="COM3", baudrate=115200, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE, parity=serial.PARITY_NONE, xonxoff=False, rtscts=False, dsrdtr=False)

    #initialize video input and color ranges
    video = 'green2.mp4'
    camera = 0
    vid = cv2.VideoCapture(0)
    malletPic = cv2.imread('pinkmalletPic.jpg')
    puckPic = cv2.imread('puckPic.jpg')

    # low_puck = np.array([40, 40, 100])
    # high_puck = np.array([80, 80, 255])
    low_mallet, high_mallet = findRange(malletPic)
    low_puck, high_puck = findRange(puckPic)

    #initialize locations
    puck_location = (0, 0)
    old_puck_location = (0, 0)
    mallet_location = (0, 0)
    old_mallet_location = (0, 0)

    prevInstruction = '0'
    
    while(True):

        ret, frame = vid.read()
        if ret == False:
            break

        frame = cv2.resize(frame, (960, 540))

        leftLimit = 80 #700
        rightLimit = 910 #835
        topLimit = 25 #105
        bottomLimit = 470 #380
               
        puck_location = findObject(frame, low_puck, high_puck)
        if not puck_location: puck_location = old_puck_location
        # if not puck_location: puck_location = (250, 250)

        mallet_location = findObject(frame, low_mallet, high_mallet)
        if not mallet_location: mallet_location = old_mallet_location


        # Compute velocity and prediction based off of previous frame
        velocity = (puck_location[0] - old_puck_location[0], puck_location[1] - old_puck_location[1])
        prediction = (puck_location[0] + velocity[0], puck_location[1] + velocity[1]) 

        old_puck_location = puck_location
        old_mallet_location = mallet_location

        slope = int(velocity[1]/(velocity[0]+1e-5))
        puckLine = Line(slope, puck_location) #defines line on which puck is traveling

        prevInstruction = move(ser, puck_location, mallet_location, prevInstruction, velocity, puckLine)
        # prevInstruction = newMove(ser, puck_location, mallet_location, prevInstruction, velocity, puckLine)

                
        cv2.imshow('Frame', frame)


        if cv2.waitKey(1) == 27:
            serial_write(ser, '0')
            break

    vid.release()
    cv2.destroyAllWindows()