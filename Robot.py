import RPi.GPIO as GPIO
import numpy as np
import math
import time
import threading
import busio 
import adafruit_vl53l0x
from simple_pid import PID
from SimpleWebSocketServer import SimpleWebSocketServer, WebSocket
from ArucoPos import MyAruco
import cv2

#Constant
PULSE_LENGTH = 0.270 #mm
L = 265.5   #mm
SPEED_FACTOR = 6.2

# Port def
SDA = 2
SCL = 3
ARM_UP = 16
ARM_DOWN = 21
ARM_LEFT = 23
ARM_RIGHT = 24
ARM_FINGER_OUT = 17
ARM_FINGER_IN = 27
ARM_CLAMP_CLOSE = 25
ARM_CLAMP_OPEN = 26

LEFT_ENCODER_A = 8
LEFT_ENCODER_B = 7
LEFT_FORWARD = 12
LEFT_REVERSE = 18

RIGHT_ENCODER_A = 5
RIGHT_ENCODER_B = 6
RIGHT_FORWARD = 19
RIGHT_REVERSE = 13


# Setup pin mode
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(ARM_UP,GPIO.OUT)
GPIO.setup(ARM_DOWN,GPIO.OUT)
GPIO.setup(ARM_LEFT,GPIO.OUT)
GPIO.setup(ARM_RIGHT,GPIO.OUT)
GPIO.setup(ARM_FINGER_OUT,GPIO.OUT)
GPIO.setup(ARM_FINGER_IN,GPIO.OUT)
GPIO.setup(ARM_CLAMP_CLOSE,GPIO.OUT)
GPIO.setup(ARM_CLAMP_OPEN,GPIO.OUT)

GPIO.setup(LEFT_FORWARD,GPIO.OUT)
GPIO.output(LEFT_FORWARD, GPIO.LOW)
GPIO.setup(LEFT_REVERSE,GPIO.OUT)
GPIO.output(LEFT_REVERSE, GPIO.LOW)
GPIO.setup(RIGHT_FORWARD,GPIO.OUT)
GPIO.output(RIGHT_FORWARD, GPIO.LOW)
GPIO.setup(RIGHT_REVERSE,GPIO.OUT)
GPIO.output(RIGHT_REVERSE, GPIO.LOW)

GPIO.setup(LEFT_ENCODER_A,GPIO.IN)
GPIO.setup(LEFT_ENCODER_B,GPIO.IN)
GPIO.setup(RIGHT_ENCODER_A,GPIO.IN)
GPIO.setup(RIGHT_ENCODER_B,GPIO.IN)

# Setup PWM for motor controlling
freq = 200
leftF = GPIO.PWM(LEFT_FORWARD, freq)
leftR = GPIO.PWM(LEFT_REVERSE, freq)
rightF = GPIO.PWM(RIGHT_FORWARD, freq)
rightR = GPIO.PWM(RIGHT_REVERSE, freq)

# Setup range sensor
i2c = busio.I2C(SCL, SDA)
rangeSensor = adafruit_vl53l0x.VL53L0X(i2c)

# Control function
def arm_up():
    GPIO.output(ARM_UP,GPIO.HIGH)
    GPIO.output(ARM_DOWN,GPIO.LOW)

def arm_down():
    GPIO.output(ARM_UP,GPIO.LOW)
    GPIO.output(ARM_DOWN,GPIO.HIGH)

def arm_left():
    GPIO.output(ARM_RIGHT,GPIO.LOW)
    GPIO.output(ARM_LEFT,GPIO.HIGH)

def arm_right():
    GPIO.output(ARM_LEFT,GPIO.LOW)
    GPIO.output(ARM_RIGHT,GPIO.HIGH)

def arm_finger_out():
    GPIO.output(ARM_FINGER_IN,GPIO.LOW)
    GPIO.output(ARM_FINGER_OUT,GPIO.HIGH)
    
def arm_finger_in():
    GPIO.output(ARM_FINGER_OUT,GPIO.LOW)
    GPIO.output(ARM_FINGER_IN,GPIO.HIGH)

def arm_clamp_open():
    GPIO.output(ARM_CLAMP_CLOSE,GPIO.LOW)
    GPIO.output(ARM_CLAMP_OPEN,GPIO.HIGH)
    
def arm_clamp_close():
    GPIO.output(ARM_CLAMP_OPEN,GPIO.LOW)
    GPIO.output(ARM_CLAMP_CLOSE,GPIO.HIGH)
    
def arm_stop():
    GPIO.output(ARM_UP,GPIO.LOW)
    GPIO.output(ARM_DOWN,GPIO.LOW)
    GPIO.output(ARM_RIGHT,GPIO.LOW)
    GPIO.output(ARM_LEFT,GPIO.LOW)
    GPIO.output(ARM_FINGER_IN,GPIO.LOW)
    GPIO.output(ARM_FINGER_OUT,GPIO.LOW)
    GPIO.output(ARM_CLAMP_OPEN,GPIO.LOW)
    GPIO.output(ARM_CLAMP_CLOSE,GPIO.LOW)

# Global variable
curPos = [0.0,0.0,0.0]
initMap={}
class armInit(threading.Thread):    
    def run(self):
        # Setup arm to init position
        arm_down()
        arm_left()
        arm_finger_in()
        arm_clamp_open()
        time.sleep(13)
        arm_stop()

def errAngle(ref,cur):
    if abs(cur - ref) > np.pi:
        if (cur - ref) > 0:
            return (cur - ref) - 2*np.pi
        else:
            return (cur - ref) + 2*np.pi
    else:
        return (cur - ref)
    
    
def stdOrientation(angle):  #angle to orientation (-pi to pi)
    while abs(angle) > 2*np.pi:
        if angle > 0:
            angle -= 2*np.pi
        else:
            angle += 2*np.pi
    
    if abs(angle) <= np.pi:
        return angle
    elif angle > np.pi:
        return angle - 2*np.pi
    else:
        return angle + 2*np.pi

class Obj:
    def __init__(self,ID, pos, length, depth):
        self.ID = ID
        self.pos = pos
        self.length = length
        self.depth = depth
        
class Point:
    def __init__(self,x,y):
        self.x = x
        self.y = y
        
def getBook():
    arm_right()
    tic = time.clock()
    while time.clock()-tic < 13:        
        dis2Book = rangeSensor.range
        arm_right()
        #print(dis2Book)
        if dis2Book < 80:
            break;
        time.sleep(0.1)
    arm_stop()
    
    arm_up()
    tic = time.clock()
    while time.clock()-tic < 7:        
        dis2Book = rangeSensor.range
        #print(dis2Book)
        time.sleep(0.1)
        
        if dis2Book > 150:
            break;
        time.sleep(0.1)    
    arm_stop()
    
    arm_finger_out()
    time.sleep(3)
    arm_stop()
    
    arm_down()
    time.sleep(5)
    arm_stop()
    arm_finger_in()
    time.sleep(5)
    arm_stop()
    arm_clamp_close()
    time.sleep(5)
    arm_stop()
    arm_up()
    time.sleep(2)
    arm_stop()
    arm_left()
    time.sleep(13)
    arm_stop()
    arm_down()
    
    
leftDistance = np.int64(0)
rightDistance = np.int64(0)
dx = [0.0,0.0]
curPos = [0,0,0]
initPos = [0,0,0]
arucoPos = None
arucoUpdate = True
bookID = 1000
leftSpeed = 0 # range from -100 to 100
rightSpeed = 0 # # range from -100 to 100
threadRun = True

class encoder(threading.Thread):    
    def run(self):
        global leftDistance
        global rightDistance
        global dx 
        last = [GPIO.input(LEFT_ENCODER_A),GPIO.input(LEFT_ENCODER_B),GPIO.input(RIGHT_ENCODER_A),GPIO.input(RIGHT_ENCODER_B)]
        while threadRun:
            cur = [GPIO.input(LEFT_ENCODER_A),GPIO.input(LEFT_ENCODER_B),GPIO.input(RIGHT_ENCODER_A),GPIO.input(RIGHT_ENCODER_B)]
            # left encoder
            if last[0] == 0 and cur[0] == 1: 
                if cur[1] == 0:
                    leftDistance +=1
                    dx[0] +=1
                else:
                    leftDistance -=1
                    dx[0]-=1
            elif last[0] == 1 and cur[0] == 0: 
                if cur[1] == 1:
                    leftDistance +=1
                    dx[0]+=1
                else:
                    leftDistance -=1
                    dx[0]-=1
            # right encoder
            if last[2] == 0 and cur[2] == 1: 
                if cur[3] == 0:
                    rightDistance +=1
                    dx[1] +=1
                else:
                    rightDistance -=1
                    dx[1]-=1
            elif last[2] == 1 and cur[2] == 0: 
                if cur[3] == 1:
                    rightDistance +=1
                    dx[1]+=1
                else:
                    rightDistance -=1
                    dx[1]-=1
            last = cur



class speedRegulator(threading.Thread):
    
    def run(self):
        dt = 0.05
        kp = 0.8
        ki = 2.5
        kd = 0.01
        pid_left = PID(kp,ki,kd, setpoint=0)
        pid_right = PID(kp,ki,kd, setpoint=0)
        pid_left.sample_time = dt
        pid_right.sample_time = dt

        while threadRun:
            last_leftDistance = leftDistance
            last_rightDistance = rightDistance
            time.sleep(dt)
            if leftDistance==0:
                delta_leftDistance = 0
            else:
                delta_leftDistance = leftDistance - last_leftDistance
            if rightDistance ==0:
                delta_rightDistance = 0
            else:
                delta_rightDistance = rightDistance - last_rightDistance
            cur_leftSpeed = delta_leftDistance/(23*dt)
            cur_rightSpeed = delta_rightDistance/(23*dt)
            left_err = abs(cur_leftSpeed) - abs(leftSpeed)
            right_err = abs(cur_rightSpeed) - abs(rightSpeed)
            pid_left.output_limits = (-abs(leftSpeed), 100-abs(leftSpeed))
            pid_right.output_limits = (-abs(rightSpeed), 100-abs(rightSpeed))
            left_fix = pid_left(left_err)
            right_fix =  pid_right(right_err)
            
            if leftSpeed == 0:
                leftR.stop()
                leftF.stop()
            elif leftSpeed > 0:
                leftR.stop()
                leftF.start(leftSpeed+left_fix)                
            else:
                leftF.stop()
                leftR.start(-leftSpeed + left_fix)
            
            if rightSpeed == 0:
                rightR.stop()
                rightF.stop()
            elif rightSpeed > 0:
                rightR.stop()
                rightF.start(rightSpeed + right_fix)                
            else:
                rightF.stop()
                rightR.start(-rightSpeed + right_fix)
                
    
                

straight = True
class keepStraight(threading.Thread):
    
    def run(self):
        global leftSpeed
        global rightSpeed
        global dx
        global curPos
        global curPos_d
        dt = 0.1
        pid_straight = PID(0.5,0.5,0.04, setpoint=0)
        pid_straight.sample_time = dt
        flag = False
        
        while threadRun:
            if straight and leftSpeed !=0 and rightSpeed !=0:
                if flag == False:
                    pid_straight.set_auto_mode = True                    
                    flag = True
                err = (abs(leftDistance) - abs(rightDistance))*PULSE_LENGTH            
                pid_straight.output_limits = (-speed/2, speed/2)
                fix = pid_straight(err)
                #print(leftDistance, rightDistance, fix)        
                if leftSpeed > 0:                    
                    leftSpeed = speed + fix                
                elif leftSpeed < 0:
                    leftSpeed = -speed - fix            
                
                if rightSpeed > 0:
                    rightSpeed = speed - fix                
                elif rightSpeed < 0:
                    rightSpeed = -speed + fix
            else:
                if flag == True:
                    pid_straight.set_auto_mode = False
                    flag = False                   
            
            time.sleep(dt)



#Position calculation
class pose(threading.Thread):    
    def run(self):        
        global dx
        global curPos
        
        dt= 0.005
        while threadRun:                  
            # Compute for the endoder position
            dx = PULSE_LENGTH * np.array(dx)  
            B = ([math.cos(curPos[2])/2,math.cos(curPos[2])/2],[math.sin(curPos[2])/2,math.sin(curPos[2])/2],[-1/L,1/L])
            curPos[0] = curPos[0] + np.dot(B,dx)[0]
            curPos[1] = curPos[1] + np.dot(B,dx)[1]            
            curPos[2] = stdOrientation(curPos[2] + np.dot(B,dx)[2])
            if abs(curPos[2]) > np.pi:
                print("ERRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR")
            dx=[0,0]
#            # Dynamic model position
#            dxx = SPEED_FACTOR*np.array([leftSpeed,rightSpeed])
#            dxx = dt*dxx
#            B = ([math.cos(curPos_d[2])/2,math.cos(curPos_d[2])/2],[math.sin(curPos_d[2])/2,math.sin(curPos_d[2])/2],[-1/L,1/L])
#            curPos_d = curPos_d + np.dot(B,dxx)
            #print(dxx)
            time.sleep(dt)           

speed = 30
#Move function
 

def stop():
    global leftSpeed
    global rightSpeed
    global leftDistance
    global rightDistance
    leftSpeed = 0
    rightSpeed = 0
    
def forward():
    global leftSpeed
    global rightSpeed
    global leftDistance
    global rightDistance
    leftDistance = 0
    rightDistance = 0
    leftSpeed = speed
    rightSpeed = speed
def backward():
    global leftSpeed
    global rightSpeed
    global leftDistance
    global rightDistance
    leftDistance = 0
    rightDistance = 0
    leftSpeed = -speed
    rightSpeed = -speed
def left():
    global leftSpeed
    global rightSpeed
    global leftDistance
    global rightDistance
    leftDistance = 0
    rightDistance = 0
    leftSpeed = -speed
    rightSpeed = speed
def right():
    global leftSpeed
    global rightSpeed
    global leftDistance
    global rightDistance
    leftDistance = 0
    rightDistance = 0
    leftSpeed = speed
    rightSpeed = -speed
    
#def dis2Speed(curDistance,distance,maxSpeed):
#    if maxSpeed < 30:
#        return maxSpeed
#    if curDistance < 100:
#        return min(30+0.7*curDistance, maxSpeed)
#    elif curDistance > 100 and curDistance < distance -300:
#        return maxSpeed
#    else:
#        return max((distance-curDistance)*maxSpeed/300,30)


    
#def angle2Speed(curAngle,angle):
#    maxSpeed = 35
#    if angle < 30:
#        return 15
#    if curAngle > angle - 20:
#        return max((angle-curAngle)*maxSpeed/20,15)
#    else:
#        return maxSpeed
    

#def turn(angle):
#    angle = angle*np.pi/180
#    global leftSpeed
#    global rightSpeed    
#    global straight
#    global speed
#    global arucoUpdate
#    arucoUpdate = True
#    speed = 20
#    lastAngle = curPos[2]
#    mem = straight
#    straight = True
#    print(angle)
#    if angle > 0:
#        left()            
#    else:        
#        right()
#    while 1:
#        curAngle = abs(errAngle(lastAngle,curPos[2]))
#        print(lastAngle,curPos[2],curAngle)
#        #speed = angle2Speed(curAngle,abs(angle))
#        #print(speed)
#        if curAngle + 5*np.pi/180 > abs(angle):
#            print("sadfgasdfFA")
#            stop()
#            ss = 50
#            if angle > 0:                
#                leftF.start(ss)
#                leftR.stop()
#                rightF.stop()
#                rightR.start(ss)                
#            else:
#                leftR.start(ss)
#                leftF.stop()
#                rightR.stop()
#                rightF.start(ss)
#            time.sleep(0.3)
#            leftF.stop()
#            leftR.stop()
#            rightF.stop()
#            rightR.stop()
#            straight = mem
#            break;
#         
#        time.sleep(0.01)
        
def dis2Pos(cur, des, disMode=0):
#    pos =[0,0,0]
#    pos[0]=des[0]
#    pos[1]=des[1]
##    if des != desPos:
##        direction = math.atan2(pos[1] - cur[1], pos[0] - cur[0])
##        pos[2]=direction + np.pi/2
##    else:
#    pos[2]=des[2]
#        
#    if disMode == 0:        
#        p1 = [pos[0], pos[1]]
#        p2 = [pos[0]+10*np.cos(pos[2]), pos[1]+10*np.sin(pos[2])]
#        p3 = [cur[0], cur[1]]    
#        d=abs((p2[1]-p1[1])*p3[0]-(p2[0]-p1[0])*p3[1] + p2[0]*p1[1] - p2[1]*p1[0]) /np.sqrt((p2[1]-p1[1])*(p2[1]-p1[1]) + (p2[0]-p1[0])*(p2[0]-p1[0]))
#    else:
    d = math.sqrt((des[1] - cur[1])*(des[1] - cur[1]) + (des[0] - cur[0])*(des[0] - cur[0]))    
    return d


# Create path, following path and grab book
def intersect(A,B,C,D):
    def ccw(A,B,C):
        return (C.y-A.y)*(B.x-A.x) > (B.y-A.y)*(C.x-A.x)
    return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)

def pointRotate(p, angle):
    x = p.x*math.cos(angle) + p.y*math.sin(angle)
    y = -p.x*math.sin(angle) + p.y*math.cos(angle)
    r = Point(x,y) 
    return r
    
def get4Corner(x, y, w, l, angle):
    # 4 rectangle points center at (0,0)
    A = Point(-w/2,-l/2)
    B = Point(-w/2,l/2)
    C = Point(w/2,l/2)
    D = Point(w/2,-l/2)    
    # rotate
    A = pointRotate(A,-angle)
    B = pointRotate(B,-angle)
    C = pointRotate(C,-angle)
    D = pointRotate(D,-angle)
    #translate to the x,y point
    A.x = A.x + x
    A.y = A.y + y
    B.x = B.x + x
    B.y = B.y + y
    C.x = C.x + x
    C.y = C.y + y
    D.x = D.x + x
    D.y = D.y + y    
    return A,B,C,D

def collisionCheck(A,B,w,obj):
    def check(A,B,C1,C2,C3,C4):
        if intersect(A,B,C1,C2):
            return True
        elif intersect(A,B,C2,C3):
            return True
        elif intersect(A,B,C3,C4):
            return True
        elif intersect(A,B,C4,C1):
            return True
        return False
    
    C1, C2, C3, C4 = get4Corner(obj.pos[0],obj.pos[1],obj.length,obj.depth,obj.pos[2])    
    V1, V2, V3, V4 = get4Corner((A.x+B.x)/2,(A.y+B.y)/2,400+math.sqrt((B.x-A.x)*(B.x-A.x) + (B.y-A.y)*(B.y-A.y)),w,math.atan2(B.y-A.y, B.x-A.x))
    return check(A,B,C1,C2,C3,C4) or check(V2,V3,C1,C2,C3,C4) or check(V1,V4,C1,C2,C3,C4)

def path(startPose, desPose, objList):
    path = []       
    # Go straigh line
    collision = False
    for i in objList.keys():
        if i < 11:
            if collisionCheck(Point(startPose[0],startPose[1]),Point(desPose[0],desPose[1]),300, objList[i]):
               collision = True
               break;
    if not collision:
        path.append(desPose)
        return path
    
    # L path, x first
    collision = False
    for i in objList.keys():
        if i < 11:
            if collisionCheck(Point(startPose[0],startPose[1]),Point(desPose[0],startPose[1]),300, objList[i]):
                collision = True
                break;
            elif collisionCheck(Point(desPose[0],startPose[1]),Point(desPose[0],desPose[1]),300, objList[i]):
                collision = True
                break;
    if not collision:    
        path.append([desPose[0],startPose[1],desPose[2]])
        path.append([desPose[0],desPose[1],desPose[2]])
        return path
    
    # L path, y first
    collision = False
    for i in objList.keys():
        if i < 11:
            if collisionCheck(Point(startPose[0],startPose[1]),Point(startPose[0],desPose[1]),300, objList[i]):
                collision = True
                break;
            elif collisionCheck(Point(startPose[0],desPose[1]),Point(desPose[0],desPose[1]),300, objList[i]):
                collision = True
                break;
    if not collision:
        path.append([startPose[0],desPose[1],desPose[2]])
        path.append([desPose[0],desPose[1],desPose[2]])
        return path
    
    return path

def move(distance,maxSpeed):
    global speed
    global straight
    global leftSpeed
    global rightSpeed
    mem = straight
    straight = True
    speed= maxSpeed
    if distance > 0:
        forward()
    else:
        backward()
    
    while True:
        if abs(distance - leftDistance*PULSE_LENGTH) < 5:            
            leftSpeed = 0

        
        if abs(distance - rightDistance*PULSE_LENGTH) < 5:            
            rightSpeed = 0

        if leftSpeed == 0  and rightSpeed ==0:
            break
        time.sleep(0.005)
    
    print("Done move ", distance )
    straight = mem

def searchBook(distance, maxSpeed, angle, ID = 10):
    global leftSpeed
    global rightSpeed    
    global straight
    global speed
    global arucoUpdate
    arucoUpdate = True
    ar = MyAruco()
    print("ar ok")
    time.sleep(1)
    dt = 0.1
    pid_straight = PID(4.5,1.5,0.4, setpoint=0)
    pid_straight.sample_time = dt
    mem = straight
    straight = False        
        
    speed = maxSpeed
    if distance > 0:
        forward()            
    else:        
        backward()
    while 1:
        curDistance = max(abs(leftDistance),abs(rightDistance))*PULSE_LENGTH
        #speed = dis2Speed(curDistance,abs(distance),maxSpeed)
        #print(speed)
        # Keep direction by angle
        if straight == False and leftSpeed !=0 and rightSpeed !=0:                
            err = -errAngle(angle,curPos[2])*180/np.pi
            #print(err)
            pid_straight.output_limits = (-speed/3, speed/3)
            fix = pid_straight(err)
            #print(err,fix)
            #print(leftDistance, rightDistance, fix)        
            if leftSpeed > 0:                    
                leftSpeed = speed + fix                
            elif leftSpeed < 0:
                leftSpeed = -speed + fix            
                
            if rightSpeed > 0:
                rightSpeed = speed - fix                
            elif rightSpeed < 0:
                rightSpeed = -speed - fix
        
        # Search book
        found, dis = ar.detectAndGetTargetDist(ID,1)
        if found:
            speed = 10
            if dis > 0 and leftSpeed < 0:
                leftSpeed = speed
                rightSpeed = speed
            if dis < 0 and leftSpeed > 0:
                leftSpeed = -speed
                rightSpeed = -speed 
            
            print(dis)
        
        if curDistance >= abs(distance) or (found and abs(dis)< 5):
            stop()
            ss = 100
            if distance > 0:                
                leftR.start(ss)
                leftF.stop()
                rightF.stop()
                rightR.start(ss)                
            else:
                leftF.start(ss)
                leftR.stop()
                rightR.stop()
                rightF.start(ss)
            time.sleep(0.4)
            leftF.stop()
            leftR.stop()
            rightF.stop()
            rightR.stop()
            if found:                
                print("Found and grab the book")
                turn2Dir(desPos[2])
                found, dis = ar.detectAndGetTargetDist(ID,1)
                print("Last D2Book: ",dis)
                while abs(dis) > 5:
                    move(dis,20)
                    time.sleep(0.5)
                    found, dis = ar.detectAndGetTargetDist(ID,1)
                    print("Last D2Book: ",dis)
                arucoUpdate = True
                print("Last D2Book: ",dis)
                getBook()
            else:
                print("Cannot find the book")
            break;
        
        
            
        if not found:
            time.sleep(dt)
    del ar
    straight = mem

def turn2Dir(direction, precise = False):
    global leftSpeed
    global rightSpeed    
    global straight
    mem = straight
    straight = True
    global speed
    global arucoUpdate
    arucoUpdate = True
    
    dt = 0.01
    
    angle = - errAngle(direction,curPos[2])*180/np.pi
    #print("Turn: ",angle)
    speed = 25
    if angle > 0:
        left()
    else:
        right()
    lastErr = abs(angle)
    i = 0
    derr = 0
    while True:
        i +=1
        err = abs(errAngle(direction,curPos[2])*180/np.pi)
        if i % 10 == 0:
            derr = lastErr - err
            lastErr = err
            #print("derr: ",derr)
        #print("Status: ",err,curPos[2]*180/np.pi,direction*180/np.pi)   
        
        if abs(err) < 5 or (abs(err) < 10 and derr < -4):
            print("Get direction: ",curPos[2]*180/np.pi,direction*180/np.pi)            
            stop()
            ss = 50
            if (angle) > 0:                
                leftF.start(ss)
                leftR.stop()
                rightF.stop()
                rightR.start(ss)                
            else:
                leftR.start(ss)
                leftF.stop()
                rightR.stop()
                rightF.start(ss)
            time.sleep(1)
            leftF.stop()
            leftR.stop()
            rightF.stop()
            rightR.stop()
            straight = mem
            err = abs(errAngle(direction,curPos[2])*180/np.pi) 
            if precise and err > 5:
                turn2Dir(direction)
            else:
                break;
        time.sleep(dt)

def path2actions(startPos, endPos):  
    startPos = np.array(startPos)
    endPos = np.array(endPos)
    GoFW = False
    GoBW = False
   
    action = [0,0]
    alphaFW = abs(errAngle(np.arctan2(endPos[1]-startPos[1], endPos[0]-startPos[0]), startPos[2]))
    alphaBW = abs(errAngle(stdOrientation(np.arctan2(endPos[1]-startPos[1], endPos[0]-startPos[0]) + np.pi), startPos[2]))
    
    if alphaFW <= alphaBW:
        GoFW = True
        action[0]= np.arctan2(endPos[1]-startPos[1], endPos[0]-startPos[0])
    else:
        GoBW = True
        action[0]= stdOrientation(np.arctan2(endPos[1]-startPos[1], endPos[0]-startPos[0]) + np.pi)
            
        
    # Go forward
    if GoFW:
        action[1] = np.linalg.norm(endPos - startPos, ord = 2)
    # Go backward
    if GoBW:
        action[1] =  -np.linalg.norm(endPos - startPos, ord = 2)
    return action

def goto(desPos, maxSpeed):
    global leftSpeed
    global rightSpeed    
    global straight
    mem = straight
    straight = False
    global speed
    global arucoUpdate
    arucoUpdate = True
    #print(curPos)
    #print(desPos)
    pathList = path(curPos, desPos, initMap)
    if pathList == []:
        return False
    print("Path: ", pathList)
    for p in pathList:
        ac = path2actions(curPos, p)
        #direction = math.atan2(p[1] - curPos[1], p[0] - curPos[0])
        
#        turnAngle = - errAngle(direction,curPos[2])*180/np.pi
#        print("Turn 1: ", turnAngle)
        #print(direction*180/np.pi)
        #print(curPos[2]*180/np.pi)
        direction = ac[0]
        turn2Dir(direction)    
        
        #distance = math.sqrt((desPos[1] - curPos[1])*(desPos[1] - curPos[1]) + (desPos[0] - curPos[0])*(desPos[0] - curPos[0]))    
        distance = ac[1]
        dt = 0.1
        pid_straight = PID(1.2,0.5,0.04, setpoint=0)
        pid_straight.sample_time = dt
        
        speed = maxSpeed
        if distance > 0:
            forward()            
        else:        
            backward()
        curDistance = abs(distance)
        while 1:
            ds = curDistance
            curDistance = dis2Pos(curPos, p)
            
            ds = ds - curDistance 
            if curDistance > 200:
                direction = math.atan2(p[1] - curPos[1], p[0] - curPos[0])
            if curDistance < 400:
                speed = 30
            #print(curDistance)
            
            #print(speed)
            # Keep direction by angle
            if straight == False and leftSpeed !=0 and rightSpeed !=0:                
                if distance > 0:
                    err = - errAngle(direction,curPos[2])*180/np.pi
                else:
                    err = errAngle(direction,curPos[2])*2
                pid_straight.output_limits = (-speed/3, speed/3)
                fix = pid_straight(err)
                #print(err,fix)
                #print(leftDistance, rightDistance, fix)        
                if leftSpeed > 0:                    
                    leftSpeed = speed + fix                
                elif leftSpeed < 0:
                    leftSpeed = -speed + fix            
                
                if rightSpeed > 0:
                    rightSpeed = speed - fix                
                elif rightSpeed < 0:
                    rightSpeed = -speed - fix
            print(curDistance,ds)
            if curDistance < 30 or (ds < 0 and curDistance < 300):
                stop()
                ss = 50
                if distance > 0:                
                    leftR.start(ss)
                    leftF.stop()
                    rightF.stop()
                    rightR.start(ss)                
                else:
                    leftF.start(ss)
                    leftR.stop()
                    rightR.stop()
                    rightF.start(ss)
                time.sleep(0.4)
                leftF.stop()
                leftR.stop()
                rightF.stop()
                rightR.stop()
                print("Last distance: ", dis2Pos(curPos, p))
                if curDistance > 30:
                    aa = path2actions(curPos, p)
                    turn2Dir(aa[0])
                    move(aa[1],20)
                    #goto(p,30)
                break;
            
            time.sleep(dt)
    # Last turn    
    turn2Dir(desPos[2],True)
    print("After go: ", curPos)
    straight = mem
    return True
    
def goalFromShelf(shelf, dis):
    rec_corner = [shelf.pos[0]+dis*np.cos(shelf.pos[2]+np.pi)-(shelf.depth/2 + 180)*np.cos(shelf.pos[2]-np.pi/2),
                      shelf.pos[1]+dis*np.sin(shelf.pos[2]+np.pi)-(shelf.depth/2 + 180)*np.sin(shelf.pos[2]-np.pi/2)]    
    return [rec_corner[0],rec_corner[1],shelf.pos[2]]


class SimpleEcho(WebSocket):

    def handleMessage(self):
        global curPos
        global arucoPos
        global speed
        global initPos
        global bookID
        # receive message
        info = self.data
        #print(info[0], info[1])
        if info[0] == "&":
            if info[1] == "I":
                t = info.split(",")
                print(t)
                ID = int(t[1])
                pos = [int(t[2]),int(t[3]),float(t[4])]
                length = int(t[5])
                depth = int(t[6])                
                if ID == 11:
                    curPos = pos
                    initPos = pos
                else:
                    initMap[ID] = Obj(ID, pos, length,depth)
                print(ID, pos, length, depth)
            elif info[1] == "R":
                t = info.split(",")                
                arucoPos = [int(t[1]),int(t[2]),float(t[3])]
                if arucoUpdate:
                    curPos=arucoPos
                    
            elif info[1] == "B":
                t = info.split(",")                
                if bookID == 1000:
                    bookID = int(t[1])
                    print("Get book #", bookID)
                
        elif info[0] == 126:
            speed = info[1]
            print("Set speed: ", speed)
        elif info[0]=="@":            
            dataa = str(curPos[0]) + ", " + str(curPos[1]) + ", " + str(curPos[2])
            self.sendMessage(dataa)
            print(dataa)
        
        elif info[0] == "#":
            if info[1] == "0":
                stop()
                print("stop")
            elif info[1] == "F":
                forward()
                print(leftSpeed,rightSpeed)
                
            elif info[1] == "B":
                backward()
            elif info[1] == "L":
                left()
            elif info[1] == "R":
                right()
            elif info[1] == "C":
                getBook()
            elif info[1] == "D":
                if info[2] == "R":
                    armInit().start()
        
    def handleConnected(self):
        print(self.address, 'connected')

    def handleClose(self):
        print(self.address, 'closed')

class ss(threading.Thread):    
    def run(self):
        server = SimpleWebSocketServer('', 8000, SimpleEcho)
        server.serveforever()          
                
                

try:    
    speed = 30
    #straight = False
    ee = encoder().start()
    pose_thread = pose().start()
    sss = ss().start()    
    thread_straight = keepStraight().start()
    thread_speed = speedRegulator().start()
    pose_thread = pose().start()
    
    # Wait for init map    
    while (len(initMap) == 0):
        time.sleep(0.1)
    time.sleep(2)
    while True:
        if bookID !=1000:
            if bookID < 60:
                desPos = tuple(goalFromShelf(initMap[1],initMap[1].length/2 + 500))
            else:
                desPos = tuple(goalFromShelf(initMap[2],initMap[1].length/2 + 500))
            print(desPos)    
            # Arm to initial position
            armInit().start()
            if goto(desPos,60):
                print("===================================Search Book======")
                searchBook(1200,15,desPos[2],bookID)
                turn2Dir(stdOrientation(curPos[2]+np.pi/1.6))
                move(100,50)
                print("Init pos: ", initPos)
                if not goto(initPos,100):
                    goto(desPos,30)
                    goto(initPos,100)
            bookID = 1000
        time.sleep(0.1)


#    speed = 12
#    ar = MyAruco()
#    forward()
#    ff = True
    while 1:
        time.sleep(0.1)
#        found, dis = ar.detectAndGetTargetDist(10,1)
#        if found:            
#            print("Found, Distance to target book: ", dis)          
#            #stop()
#            if abs(dis)< 3:
#                stop()
#                time.sleep(1)
#                if ff:
#                    getBook()
#                    ff= False
#                #break
#        if leftDistance*PULSE_LENGTH > 1000:
#            stop()
#            print("Cannot find the book ID 10")
        #if (curPos[2]>np.pi/2):
            #break
        #print(rangeSensor.range)
        #print(curPos[0], curPos[1], curPos[2]*180/np.pi)
        
except KeyboardInterrupt:
    pass

threadRun = False
leftF.stop()
leftR.stop()
rightF.stop()
rightR.stop()
time.sleep(1)
GPIO.cleanup()
    
            