import cv2
import math
import time

class Line: #used for trajectory
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
    
def findObject(frame, low_color, high_color): #finds object of given color

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, low_color, high_color)

    _, mask = cv2.threshold(mask, 254, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    

    if len(contours) > 0:
        # Find largest contour, draw bounding box, find center
        largest_contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest_contour) < 200:
            return None
        
        x, y, w, h = cv2.boundingRect(largest_contour)
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 3) #rectangle around object

        moments = cv2.moments(largest_contour)   
        centroid = (int(moments["m10"] / (moments["m00"] + 1e-5)), int(moments["m01"] / (moments["m00"] + 1e-5))) #find center of object
        return centroid

    else:
        return None
            
def findBounce(frame, velocity, puck, puckLine, step): #predicts location where puck will strike a wall and bounce towards

    top = Line(0, (0, 30))
    bottom = Line(0, (0, 440))
    left = Line(999, (90, 0))
    right = Line(999, (900, 0))

    if step == 0: #increase to see more collisions
        return
    
    step -= 1

    if velocity[0] >= 0 and velocity[1] >= 0: 
        #bottom and right
        bottomIntersect = puckLine.intersect(bottom) #find intersection with bottom of table
        rightIntersect = puckLine.intersect(right) #find intersection with right side of table

        if math.sqrt((puck[0] - bottomIntersect[0])**2 + (puck[1] - bottomIntersect[1])**2) < math.sqrt((puck[0] - rightIntersect[0])**2 + (puck[1] - rightIntersect[1])**2): #compare distance to bottom and distance to right
            cv2.circle(frame, bottomIntersect, radius=10, color=(0,0,255), thickness=-1) #bottom
            bounce = puckLine.reflection(bottom)
            newVelocity = (velocity[0], -1 * velocity[1])
            findBounce(frame, newVelocity, bottomIntersect, bounce, step)
            

        else:
            cv2.circle(frame, rightIntersect, radius=10, color=(0,0,255), thickness=-1) #right
            bounce = puckLine.reflection(right)
            newVelocity = (-1 * velocity[0], velocity[1])
            findBounce(frame, newVelocity, rightIntersect, bounce, step)
            
  
    elif velocity[0] < 0 and velocity[1] >= 0:
        #bottom and left
        bottomIntersect = puckLine.intersect(bottom)
        leftIntersect = puckLine.intersect(left)

        if math.sqrt((puck[0] - bottomIntersect[0])**2 + (puck[1] - bottomIntersect[1])**2) < math.sqrt((puck[0] - leftIntersect[0])**2 + (puck[1] - leftIntersect[1])**2):
            cv2.circle(frame, bottomIntersect, radius=10, color=(0,0,255), thickness=-1) #bottom
            bounce = puckLine.reflection(bottom)
            newVelocity = (velocity[0], -1 * velocity[1])
            findBounce(frame, newVelocity, bottomIntersect, bounce, step)
            

        else:
            cv2.circle(frame, leftIntersect, radius=10, color=(0,0,255), thickness=-1) #left
            bounce = puckLine.reflection(left)
            newVelocity = (-1 * velocity[0], velocity[1])
            findBounce(frame, newVelocity, leftIntersect, bounce, step)
            

    elif velocity[0] >= 0 and velocity[1] < 0:
        #top and right
        topIntersect = puckLine.intersect(top)
        rightIntersect = puckLine.intersect(right)

        if math.sqrt((puck[0] - topIntersect[0])**2 + (puck[1] - topIntersect[1])**2) < math.sqrt((puck[0] - rightIntersect[0])**2 + (puck[1] - rightIntersect[1])**2):
            cv2.circle(frame, topIntersect, radius=10, color=(0,0,255), thickness=-1) #top
            bounce = puckLine.reflection(top)
            newVelocity = (velocity[0], -1 * velocity[1])
            findBounce(frame, newVelocity, topIntersect, bounce, step)
            

        else:
            cv2.circle(frame, rightIntersect, radius=10, color=(0,0,255), thickness=-1) #right
            bounce = puckLine.reflection(right)
            newVelocity = (-1 * velocity[0], velocity[1])
            findBounce(frame, newVelocity, rightIntersect, bounce, step)
            
  
    elif velocity[0] < 0 and velocity[1] < 0:
        #top and left
        topIntersect = puckLine.intersect(top)
        leftIntersect = puckLine.intersect(left)

        if math.sqrt((puck[0] - topIntersect[0])**2 + (puck[1] - topIntersect[1])**2) < math.sqrt((puck[0] - leftIntersect[0])**2 + (puck[1] - leftIntersect[1])**2):
            cv2.circle(frame, topIntersect, radius=10, color=(0,0,255), thickness=-1) #top
            bounce = puckLine.reflection(top)
            newVelocity = (velocity[0], -1 * velocity[1])
            findBounce(frame, newVelocity, topIntersect, bounce, step)
            

        else:
            cv2.circle(frame, leftIntersect, radius=10, color=(0,0,255), thickness=-1) #left
            bounce = puckLine.reflection(left)
            newVelocity = (-1 * velocity[0], velocity[1])
            findBounce(frame, newVelocity, leftIntersect, bounce, step)

def move(ser, destination, currentPosition, prevInstruction):
    #          up
    #          -
    #      ul  -    ur
    # left-----0+++++++righ
    #      dr  +    dr
    #          +
    #         down

    leftLimit = 700 #650
    rightLimit = 835 #885
    topLimit = 105 #55
    bottomLimit = 380 #430
    sleepyTime = 0.25
    
    #check to make sure mallet is not nearing left or right edge
    if currentPosition[0] not in range(leftLimit, rightLimit):
        # serial_write(ser, '0')

        if currentPosition[0] < leftLimit:
            if prevInstruction == 'righ': return prevInstruction
            serial_write(ser, 'righ')
            return 'righ'
            # time.sleep(sleepyTime)

        if currentPosition[0] > rightLimit:
            if prevInstruction == 'left': return prevInstruction
            serial_write(ser, 'left')
            return 'left'
            # time.sleep(sleepyTime)

        # print('STOP')
        return
    
    #check to make sure mallet is not nearing top or bottom edge
    if currentPosition[1] not in range(topLimit, bottomLimit):
        # serial_write(ser, '0')

        if currentPosition[1] < topLimit:
            if prevInstruction == 'down': return prevInstruction
            serial_write(ser, 'down')
            return 'down'
            # time.sleep(sleepyTime)

        if currentPosition[1] > bottomLimit:
            if prevInstruction == 'up': return prevInstruction
            serial_write(ser, 'up')
            return 'up'
            # time.sleep(sleepyTime)

        # print('STOP')
        return
    

    

    #if puck is on left side of table
    if destination[0] < 495:
        prevInstruction = move(ser, (820, destination[1]), currentPosition, prevInstruction)
        return
    



    distance = (destination[0] - currentPosition[0], destination[1] - currentPosition[1])
    x = distance[0]
    y = distance[1]
    tolerance = 20

    if x > tolerance and y > tolerance:
        if prevInstruction == 'dr': return prevInstruction
        serial_write(ser, 'dr')
        return 'dr'
        # print('dr')
    elif x > tolerance and y < -1*tolerance:
        if prevInstruction == 'ur': return prevInstruction
        serial_write(ser, 'ur')
        return 'ur'
        # print('ur')
    elif x < -1*tolerance and y > tolerance:
        if prevInstruction == 'dl': return prevInstruction
        serial_write(ser, 'dl')
        return 'dl'
        # print('dl')
    elif x < -1*tolerance and y < -1*tolerance:
        if prevInstruction == 'ul': return prevInstruction
        serial_write(ser, 'ul')
        return 'ul'
        # print('ul')
    elif y > tolerance and x < tolerance and x > -1*tolerance :
        if prevInstruction == 'down': return prevInstruction
        serial_write(ser, 'down')
        return 'down'
        # print('down')
    elif y < -1*tolerance and x < tolerance and x > -1*tolerance:
        if prevInstruction == 'up': return prevInstruction
        serial_write(ser, 'up')
        return 'up'
        # print('up')
    elif x > tolerance and y < tolerance and y > -1*tolerance:
        if prevInstruction == 'righ': return prevInstruction
        serial_write(ser, 'righ')
        return 'righ'
        # print('righ')
    elif x < -1*tolerance and y < tolerance and y > -1*tolerance:
        if prevInstruction == 'left': return prevInstruction
        serial_write(ser, 'left')
        return 'left'
        # print('left')
    else:
        if prevInstruction == 'stay': return prevInstruction
        serial_write(ser, 'stay')
        return 'stay'
        # print('stay')

def serial_write(ser, data):
    ser.flush()
    data_to_send = str(data)
    while(len((data_to_send)) < 4):
        data_to_send = data_to_send + "0"
    data_to_send = data_to_send.encode("ascii")
    print(data_to_send)
    ser.write(data_to_send)


