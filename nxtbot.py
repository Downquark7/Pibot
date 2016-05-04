import nxt

print "connecting"
brick = nxt.locator.find_one_brick()
print brick
motorb = nxt.motor.Motor(brick, nxt.motor.PORT_B)
motorc = nxt.motor.Motor(brick, nxt.motor.PORT_C)


import numpy as np
#np = c.modules.numpy
#import copy
#import cv
import cv2
#cv2 = c.modules.cv2
import time
#import os
#import array
#import StringIO

#import evdev
#import ev3dev.auto as ev3
#ev3 = c.modules.ev3dev.auto
#import threading
#threading = c.modules.threading
#import time


side = 0
w = 640
h = 480
L_motor_speed = 0
R_motor_speed = 0
forward_speed = 0.0


search_speed = 0
y_gain = input("y gain")
area_gain = input("area gain")
turn_gain = 20000/input("object diameter in inches")
overturn_gain = 10000
accumulated_gain = 0.2
turn_error = 0
accumulated_turn = 0


def clamp(n, (minn, maxn)):
    """
    Given a number and a range, return the number, or the extreme it is closest to.
    :param n: number
    :return: number
    """
    return max(min(maxn, n), minn)


def scale(val, src, dst):
    """
    Scale the given value from the scale of src to the scale of dst.
    val: float or int
    src: tuple
    dst: tuple
    example: print scale(99, (0.0, 99.0), (-1.0, +1.0))
    """
    return (float(val - src[0]) / (src[1] - src[0])) * (dst[1] - dst[0]) + dst[0]

def scalestick(value):
    return scale(value,(-100,100),(-100,100))

def dc_clamp(value):
    return clamp(value,(-100,100))

def ac_clamp(value):
    return clamp(value,(0,255))

#left_speed = 0
#right_speed = 0
#lift_speed = 0
#other_speed = 0



# This sets up the video capture
cap = cv2.VideoCapture(0)
cap.set(3,w)
cap.set(4,h)
time.sleep(2)
#cap.set(15,-80.0)
print(cap)
target=[100,100,100]


while (True):
    success, image = cap.read()
    h, w, channels = image.shape
    #image = cv2.flip(image,-1)
    image = cv2.GaussianBlur(image,(5,5),0)
    color = map(int,image[h/2][w/2])
    image_HSV = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    colour = str(image_HSV[h/2][w/2])
    cv2.line(image,(0,h/2),(w-1,h/2),color)        
    cv2.line(image,(w/2,0),(w/2,h-1),color)
    cv2.putText(image,colour,(10,30),cv2.FONT_HERSHEY_PLAIN,1,[255,255,255])
    cv2.imshow('View',image)
    #print(colour)
    # Esc key to stop, otherwise repeat after 1 milliseconds
    key_pressed = cv2.waitKey(33)
    if key_pressed == 27:
        break

target=image_HSV[h/2][w/2]
print target
#time.sleep(2)

# Main loop
while True:
    try:
        found = False
        ret, image = cap.read()
        h, w, channels = image.shape
        #print image.shape
        #image = cv2.flip(image,-1)
        #image2 = copy.deepcopy(image) 
        #image2 = cv2.cvtColor(image2,cv2.COLOR_RGB2BGR)
        binary = cv2.GaussianBlur(image,(5,5),0)
        binary = cv2.cvtColor(binary,cv2.COLOR_BGR2HSV)
        lower_pink = np.uint8([ac_clamp(target[0]-10),ac_clamp(target[1]-50),ac_clamp(target[2]-50)])
        upper_pink = np.uint8([ac_clamp(target[0]+10),ac_clamp(target[1]+50),ac_clamp(target[2]+50)])
        kernel = np.ones((5,5),np.uint8)
        mask = cv2.inRange(binary,lower_pink,upper_pink)
        mask = cv2.erode(mask,kernel,iterations=1)
        mask = cv2.dilate(mask,kernel,iterations=1)
        contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        blob_x = w/2
        area = 0
        if len(contours)>0:
            largest = 0
            area = cv2.contourArea(contours[0])
            if len(contours)>1:
                for i in range(1,len(contours)):
                    temp_area = cv2.contourArea(contours[i])
                    if temp_area>area:
                        area=temp_area
                        largest = i
            if area > 100:
                found=True
                coords = cv2.moments(contours[largest])
                blob_x = int(coords['m10']/coords['m00'])
                blob_y = int(coords['m01']/coords['m00'])
                diam = int(np.sqrt(area)/4)
                
                cv2.circle(image,(blob_x,blob_y),diam,(0,255,0),1)
                #cv2.putText(image,str(diam),(10,30),cv2.FONT_HERSHEY_PLAIN,1,[255,255,255])
                cv2.line(image,(blob_x-2*diam,blob_y),(blob_x+2*diam,blob_y),(0,255,0),1)
                cv2.line(image,(blob_x,blob_y-2*diam),(blob_x,blob_y+2*diam),(0,255,0),1)
                boxx,boxy,boxw,boxh = cv2.boundingRect(contours[largest])
                cv2.rectangle(image,(boxx,boxy),(boxx+boxw,boxy+boxh),(0,0,255),2)
            cv2.drawContours(image,contours,largest,(255,0,0),3)
            cv2.imshow("View",image)
        if not found:
            accumulated_turn = 0
            turn_error = 0
            if side == 0:
                L_motor_speed=-search_speed
                R_motor_speed=search_speed
            else:
                L_motor_speed=search_speed
                R_motor_speed=-search_speed
        else:
            direction = blob_x -w/2
            if direction < 0:
                side = 0
                if accumulated_turn > 0:
                    accumulated_turn = 0
            else:
                side = 1
                if accumulated_turn > 0:
                    accumulated_turn = 0
            
            forward_speed = ((h - (boxx + boxh + (h / 4))) * y_gain) + (area_gain * ((h*w*0.5) - area) / (h*w))
            turn_error = (direction * 2) / w
            accumulated_turn = accumulated_turn + (accumulated_gain * direction * turn_gain * 2) / w
            print ((((area/(h*w)) * direction*turn_gain) * 2) / w), (accumulated_turn*2)/w, (-turn_error*2)/w
            L_motor_speed = forward_speed + ((((area/(h*w)) * direction * turn_gain + accumulated_turn - turn_error) * 2) / w)
            R_motor_speed = forward_speed - ((((area/(h*w)) * direction * turn_gain + accumulated_turn - turn_error) * 2) / w)

            if abs(L_motor_speed) < 10:
                L_motor_speed = 0
            if abs(R_motor_speed) < 10:
                R_motor_speed = 0
            #print(error*porportional_gain, intergral, derivative*derivative_gain)
            #print("found; direction=",direction,"turning_rate",turning_rate,"w",w)
            
        found = False
        motorb.run(power=dc_clamp(L_motor_speed),regulated=True)
        motorc.run(power=dc_clamp(R_motor_speed),regulated=True)
        #time.sleep(0.01)
        #print("camera update")
    except KeyboardInterrupt:
        #motorb.run(power=0,regulated=False)
        #motorc.run(power=0,regulated=False)
        break
    key_pressed = cv2.waitKey(33)
    if key_pressed ==27:
        #motorb.run(power=0,regulated=False)
        #motorc.run(power=0,regulated=False)
        break
motorb.run(power=0,regulated=False)
motorc.run(power=0,regulated=False)
