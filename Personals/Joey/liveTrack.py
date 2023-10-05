import cv2
import numpy as np
import time
import math

video = 'airhockey.mp4'
camera = 1
vid = cv2.VideoCapture(0)

low_green = np.array([60, 40, 80])
high_green = np.array([100, 255, 255])
old_centroid = (0, 0)
velocity = (0, 0)
prediction = (0, 0)
centroid = (0, 0)
count = 0
global previousIntersections
global previousAverages
previousIntersections = []
previousAverages = []


historyFrames = 1 #adjust for number of frames in intersection history
bounceNum = 3 #adjust for number of predicted bounces
historyTolerance = 50 #adjust for tolerance in intersection decision (smaller for less detections)

class Line:
    def __init__(self, slope, point):
        self.m = slope
        x = point[0]
        y = point[1]

        self.b = -1 * slope * x + y
        self.point1 = point
        self.point2 = (point[0] + 1, int(point[1] + self.m))

    def findY(self, x):
        return self.m * x + self.b
        

    def intersect(self, otherLine):
        if self.m != otherLine.m:
            x = int((otherLine.b - self.b) / (self.m - otherLine.m))
        else:
            x = 999
        y = int(self.findY(x))
        return (x, y)
    def reflection(self, wall):
        
        intersection = self.intersect(wall)
        newSlope = -1 * self.m
        newLine = Line(newSlope, intersection)
                
        return newLine
    

def listAverage(intersections):
    newList = [i[1] for i in intersections]
    average = sum(newList) / len(newList)
    #print(f'Average: {int(average)}  Last Point: {intersections[len(intersections)-1]}')
    if abs(average - intersections[len(intersections)-1][1]) < historyTolerance:
        previousAverages.append(average)
        if len(previousAverages) > historyFrames:
            previousAverages.pop(0)
        return True
    else:
        return False

def checkPotential(puck, velocity, path, threshold, top, bottom): #checks if puck will cross some danger line
    if velocity[0] > 0:
        intersection = path.intersect(threshold)

        if intersection[1] > top and intersection[1] < bottom:      
            
            return intersection



def findBounce(velocity, puck, puckLine, step): #predicts location where puck will strike a wall and bounce towards

    if step == bounceNum: #increase to see more collisions
        return
    
    step += 1

    dangerPoint = checkPotential(puck, velocity, puckLine, Line(999, (990, 0)), 90, 625)
    if dangerPoint is not None:

        previousIntersections.append(dangerPoint)

        if len(previousIntersections) > historyFrames:
            previousIntersections.pop(0)

        temp = [i[1] for i in previousIntersections]
        average = (990, int(sum(temp) / len(temp)))

        if listAverage(previousIntersections):
            cv2.line(frame, puck, average, (0, 0, 255), 2) 
            cv2.circle(frame, average, radius=15, color=(0, 255, 0), thickness=-1)
            print(f'Move motor to: {average}')


    if velocity[0] >= 0 and velocity[1] >= 0: 
        #bottom and right
        bottomIntersect = puckLine.intersect(bottom) #find intersection with bottom of table
        rightIntersect = puckLine.intersect(right) #find intersection with right side of table

        if math.sqrt((puck[0] - bottomIntersect[0])**2 + (puck[1] - bottomIntersect[1])**2) < math.sqrt((puck[0] - rightIntersect[0])**2 + (puck[1] - rightIntersect[1])**2): #compare distance to bottom and distance to right
            #cv2.circle(frame, bottomIntersect, radius=10, color=(0,0,255), thickness=-1) #bottom
            bounce = puckLine.reflection(bottom)
            newVelocity = (velocity[0], -1 * velocity[1])
            findBounce(newVelocity, bottomIntersect, bounce, step)
            

        else:
            #cv2.circle(frame, rightIntersect, radius=10, color=(0,0,255), thickness=-1) #right
            bounce = puckLine.reflection(right)
            newVelocity = (-1 * velocity[0], velocity[1])
            findBounce(newVelocity, rightIntersect, bounce, step)
            
  
    elif velocity[0] < 0 and velocity[1] >= 0:
        #bottom and left
        bottomIntersect = puckLine.intersect(bottom)
        leftIntersect = puckLine.intersect(left)

        if math.sqrt((puck[0] - bottomIntersect[0])**2 + (puck[1] - bottomIntersect[1])**2) < math.sqrt((puck[0] - leftIntersect[0])**2 + (puck[1] - leftIntersect[1])**2):
            #cv2.circle(frame, bottomIntersect, radius=10, color=(0,0,255), thickness=-1) #bottom
            bounce = puckLine.reflection(bottom)
            newVelocity = (velocity[0], -1 * velocity[1])
            findBounce(newVelocity, bottomIntersect, bounce, step)
            

        else:
            #cv2.circle(frame, leftIntersect, radius=10, color=(0,0,255), thickness=-1) #left
            bounce = puckLine.reflection(left)
            newVelocity = (-1 * velocity[0], velocity[1])
            findBounce(newVelocity, leftIntersect, bounce, step)
            

    elif velocity[0] >= 0 and velocity[1] < 0:
        #top and right
        topIntersect = puckLine.intersect(top)
        rightIntersect = puckLine.intersect(right)

        if math.sqrt((puck[0] - topIntersect[0])**2 + (puck[1] - topIntersect[1])**2) < math.sqrt((puck[0] - rightIntersect[0])**2 + (puck[1] - rightIntersect[1])**2):
            #cv2.circle(frame, topIntersect, radius=10, color=(0,0,255), thickness=-1) #top
            bounce = puckLine.reflection(top)
            newVelocity = (velocity[0], -1 * velocity[1])
            findBounce(newVelocity, topIntersect, bounce, step)
            

        else:
            #cv2.circle(frame, rightIntersect, radius=10, color=(0,0,255), thickness=-1) #right
            bounce = puckLine.reflection(right)
            newVelocity = (-1 * velocity[0], velocity[1])
            findBounce(newVelocity, rightIntersect, bounce, step)
            
  
    elif velocity[0] < 0 and velocity[1] < 0:
        #top and left
        topIntersect = puckLine.intersect(top)
        leftIntersect = puckLine.intersect(left)

        if math.sqrt((puck[0] - topIntersect[0])**2 + (puck[1] - topIntersect[1])**2) < math.sqrt((puck[0] - leftIntersect[0])**2 + (puck[1] - leftIntersect[1])**2):
            #cv2.circle(frame, topIntersect, radius=10, color=(0,0,255), thickness=-1) #top
            bounce = puckLine.reflection(top)
            newVelocity = (velocity[0], -1 * velocity[1])
            findBounce(newVelocity, topIntersect, bounce, step)
            

        else:
            #cv2.circle(frame, leftIntersect, radius=10, color=(0,0,255), thickness=-1) #left
            bounce = puckLine.reflection(left)
            newVelocity = (-1 * velocity[0], velocity[1])
            findBounce(newVelocity, leftIntersect, bounce, step)
            
      

#defines corners of table (will do this automatically with real camera)
top = Line(0, (0, 90))
bottom = Line(0, (0, 625))
left = Line(999, (170, 0))
right = Line(999, (1075, 0))

#ASSUME RIGHT SIDE IS ROBOT SIDE
scoreLine = Line(999, (1075, 0))
        

while(True):
    ret, frame = vid.read()
    

    
    if ret == True:
        
        # Convert frame to HSV spectrum, mask using desired color range
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, low_green, high_green)
        #cv2.imshow("color", mask)
    
        # Make all non-white colors black, find contours
        _, mask = cv2.threshold(mask, 254, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if count % 120 == 0:
            #retrack table
            track = 0

        if len(contours) > 0:
            # Find largest contour, draw bounding box, find center
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)

            # for cnt in contours:
            #     x, y, w, h = cv2.boundingRect(cnt)
            #     cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 3)

            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 3) #rectangle around puck

            cv2.circle(frame, (1075, 625), radius=5, color=(255,0,0), thickness=-1) #bottom right
            cv2.circle(frame, (1075, 90), radius=5, color=(255,0,0), thickness=-1) #top right
            cv2.circle(frame, (170, 90), radius=5, color=(255,0,0), thickness=-1) #top left
            cv2.circle(frame, (170, 625), radius=5, color=(255,0,0), thickness=-1) #bottom left

            moments = cv2.moments(largest_contour)
            if moments["m00"] != 0:
                centroid = (int(moments["m10"] / moments["m00"]), int(moments["m01"] / moments["m00"]))
    
            # print(f'Velocity: {velocity}')
            # print(f'Prediction: {prediction}')
            # print(f'Actual: {centroid}\n\n')
                
            # Compute velocity and prediction based off of previous frame
            velocity = (centroid[0] - old_centroid[0], centroid[1] - old_centroid[1])
            prediction = (centroid[0] + velocity[0], centroid[1] + velocity[1]) 
            old_centroid = centroid

            #cv2.line(frame, centroid, prediction, (0,255,0), 2)


            if velocity[0] != 0:
                slope = velocity[1]/velocity[0]
            else:
                slope = 999

            puckLine = Line(slope, centroid) #defines line on which puck is traveling
            #findBounce(velocity, centroid, puckLine, 0)
            
    count += 1

    cv2.imshow('Frame', frame)
    #time.sleep(0.04)
    #time.sleep(0.5)

    if cv2.waitKey(1) == 27:
        break

vid.release()
cv2.destroyAllWindows()


    
