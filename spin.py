
import numpy as np
#import copy
#import cv
import cv2
import time
#import os
#import StringIO
#from BrickPi import *   #import BrickPi.py file to use BrickPi operations
import threading
import evdev
import ev3dev.auto as ev3

L_motor_speed = 0
R_motor_speed = 0

side = 0
w=80
h=60
turning_rate = 60
running = True
time_limit=60

startTime=time.time()

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
    return scale(value,(-255,255),(-100,100))

def dc_clamp(value):
    return clamp(value,(-100,100))

class MotorThread(threading.Thread):
    def __init__(self):
        #self.a_motor = ev3.LargeMotor(ev3.OUTPUT_A)
        self.b_motor = ev3.LargeMotor(ev3.OUTPUT_B)
        self.c_motor = ev3.LargeMotor(ev3.OUTPUT_C)
        #self.d_motor = ev3.MediumMotor(ev3.OUTPUT_D)
        time.sleep(1)
        threading.Thread.__init__(self)
    
    def run(self):
        print "Engines running!"
        while running:
            #self.a_motor.run_forever(duty_cycle_sp = dc_clamp(lift_speed))
            self.b_motor.run_forever(duty_cycle_sp = dc_clamp(scalestick(L_motor_speed)))
            self.c_motor.run_forever(duty_cycle_sp = dc_clamp(scalestick(R_motor_speed)))
            #self.d_motor.run_forever(duty_cycle_sp = dc_clamp(other_speed))
            time.sleep(0.1)
            #print("motor_speeds: ",L_motor_speed," | ",R_motor_speed)
        
        #self.a_motor.stop()
        self.b_motor.stop()
        self.c_motor.stop()
        #self.d_motor.stop()

if __name__ == "__main__":
    motor_thread = MotorThread()
    motor_thread.setDaemon(True)
    motor_thread.start()





# This sets up the video capture
cap = cv2.VideoCapture(0)
cap.set(3,w)
cap.set(4,h)
time.sleep(2)
#cap.set(15,-80.0)
seeking=True
seekTime=time.time()
# Main loop
while True and (time.time()<startTime+time_limit):
    try:
        found = False
        ret, image = cap.read()
        print image.shape()
        #image = cv2.flip(image,-1)
        #image2 = copy.deepcopy(image) 
        #image2 = cv2.cvtColor(image2,cv2.COLOR_RGB2BGR)
        binary = cv2.GaussianBlur(image,(5,5),0)
        binary = cv2.cvtColor(binary,cv2.COLOR_BGR2HSV)
        lower_pink = np.array([90,50,50])
        upper_pink = np.array([110,255,255])
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
                print "found"
                print coords
                blob_x = int(coords['m10']/coords['m00'])
                blob_y = int(coords['m01']/coords['m00'])
                diam = int(np.sqrt(area)/4)
                print(blob_x, blob_y, diam)
                #cv2.circle(image,(blob_x,blob_y),diam,(0,255,0),1)
                #cv2.line(image,(blob_x-2*diam,blob_y),(blob_x+2*diam,blob_y),(0,255,0),1)
                #cv2.line(image,(blob_x,blob_y-2*diam),(blob_x,blob_y+2*diam),(0,255,0),1)
            #cv2.drawContours(image,contours,largest,(255,0,0),3)

        if not found:
            if seeking == False:
                L_motor_speed=0
                R_motor_speed=0
            else:
                if time.time()<seekTime+10:
                    if side == -1:
                        L_motor_speed=-70
                        R_motor_speed=70
                    else:
                        L_motor_speed=70
                        R_motor_speed=-70
                else:
                    seeking=False
        else:
            seeking=True
            seekTime=time.time()
            direction = blob_x -w/2
            if direction <0:
                side = -1
            else:
                side = 1
            L_motor_speed=220*side*direction/w
            R_motor_speed=-220*side*direction/w
        found = False
    except KeyboardInterrupt:
        break
    key_pressed = cv2.waitKey(33)
    if key_pressed ==27:
        break
cap.release()
waiting = false
running = false
time.sleep(1)


