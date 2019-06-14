import numpy as np
import cv2
import cv2.aruco as aruco
import glob
import time
import yaml
import math
from numpy import cos, sin
from Map import *
import threading

#from SimpleWebSocketServer import SimpleWebSocketServer, WebSocket
#import threading
#import websocket
#try:
#    import thread
#except ImportError:
#    import _thread as thread
#import time
#
#readyToSend = False
#
#def on_message(ws, message):
#    print(message)
#
#def on_error(ws, error):
#    print(error)
#
#def on_close(ws):
#    print("### closed ###")
#
#def on_open(ws):
#    def run(*args):
#        global readyToSend
#        while True:
#            if readyToSend:
#                ws.send(str(curPos))
#                readyToSend = False
#            
#    thread.start_new_thread(run, ())
#
#class clientRun(threading.Thread):
#    def run(self):
#        websocket.enableTrace(True)
#        ws = websocket.WebSocketApp("ws://192.168.0.36:8000/",
#                                on_message = on_message,
#                                on_error = on_error,
#                                on_close = on_close)
#        ws.on_open = on_open
#        ws.run_forever()
#clientRun().start()

m = Map([-1000,3000],[-2000,2000])
curPos=[0,0,0]
objs = {}

cap = cv2.VideoCapture(0) #cv2.CAP_DSHOW + 0
a = cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)
b = cap.set(cv2.CAP_PROP_EXPOSURE, -5)
 
print(a,b)
cap.set(3,800)
cap.set(4,600)

# termination criteria
#criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

## Arrays to store object points and image points from all the images.
#objpoints = [] # 3d point in real world space
#imgpoints = [] # 2d points in image plane.

with open('calibration.yaml') as f:
    loadeddict = yaml.load(f)

mtx = np.array(loadeddict.get('camera_matrix'))
dist = np.array(loadeddict.get('dist_coeff'))
print(mtx)

def relativeAngle2D(rvec1, tvec1, rvec2, tvec2, mtx, dist):
    axis = np.float32([[0,0,0],[1,0,0], [0,1,0], [0,0,1]]).reshape(-1,3)
    imgpts, jac = cv2.projectPoints(axis, rvec1, tvec1, mtx, dist)
    imgpts = imgpts.reshape(4,2)    
    d1 = math.atan2(imgpts[1][1] - imgpts[0][1], imgpts[1][0] -imgpts[0][0])
    
    imgpts, jac = cv2.projectPoints(axis, rvec2, tvec2, mtx, dist)
    imgpts = imgpts.reshape(4,2)
    d2 = math.atan2(imgpts[1][1] - imgpts[0][1], imgpts[1][0] -imgpts[0][0])
    return (d1-d2)

def corners2Angle(corners):
    corners = corners.reshape(4,2)
    centre = corners[0] + corners[1] + corners[2] + corners[3]
    centre[:] = [int(x / 4) for x in centre]
    d = math.atan2(corners[1][1]-corners[0][1],corners[1][0]-corners[0][0])
    orientedPoint = tuple([int(50*math.cos(d) + centre[0]), int(50*math.sin(d) + centre[1])])
    return d, tuple(centre), orientedPoint

def errAngle(cur,ref):
    if abs(cur - ref) > np.pi:
        return (cur - ref) + 2*np.pi
    else:
        return (cur - ref)
    
originID = 0
originAngle = None
firstRvec = None
firstTvec = None

st = time.time()
ret, frame = cap.read()
def tracking(frame):
    global firstRvec
    global firstTvec
    global originAngle
    objj = []
    # operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #frame = cv2.flip(frame,1)
    aruco_dict = aruco.Dictionary_get(3)
    parameters = aruco.DetectorParameters_create()
    #parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX

    #lists of ids and the corners beloning to each id
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)        
    #print(ids)
    font = cv2.FONT_HERSHEY_SIMPLEX #font for displaying text (below)

    if np.all(ids != None):
        t = None
        r = None
        for i in range (0, len(ids)):
            rvec, tvec,_ = aruco.estimatePoseSingleMarkers(corners[i], 0.117, mtx, dist) #Estimate pose of each marker and return the values rvet and tvec---different from camera coefficients
            if (ids[i] == originID):
                firstRvec = rvec
                originAngle,centre,orient_centre = corners2Angle(corners[i])                
                firstTvec = tvec     
                
                #aruco.drawAxis(frame, mtx, dist, rvec, tvec, 0.5) #Draw Axis
                cv2.line(frame,centre,orient_centre,(255,0,0),4)
                c = corners[i].reshape(4,2)
                cv2.line(frame,tuple(c[0]),tuple(c[1]),(0,255,0),1)
                cv2.line(frame,tuple(c[1]),tuple(c[2]),(0,255,0),1)
                cv2.line(frame,tuple(c[2]),tuple(c[3]),(0,255,0),1)
                cv2.line(frame,tuple(c[3]),tuple(c[0]),(0,255,0),1)
                cv2.putText(frame, "Origin angle: " + str(int(originAngle*180/np.pi)), (0,575), font, 1, (0,0,255), 2, cv2.LINE_AA)
            elif (firstRvec is not None and ids[i]<12 and ids[i] != originID):
                curD,centre,orient_centre = corners2Angle(corners[i])
                cv2.line(frame,centre,orient_centre,(255,0,0),4)
                c = corners[i].reshape(4,2)
                cv2.line(frame,tuple(c[0]),tuple(c[1]),(0,255,0),1)
                cv2.line(frame,tuple(c[1]),tuple(c[2]),(0,255,0),1)
                cv2.line(frame,tuple(c[2]),tuple(c[3]),(0,255,0),1)
                cv2.line(frame,tuple(c[3]),tuple(c[0]),(0,255,0),1)
                t = tvec - firstTvec
                t = t.reshape(3,1)
                x = t[0]*math.cos(originAngle) + t[1]*math.sin(originAngle)
                y = -t[0]*math.sin(originAngle) + t[1]*math.cos(originAngle)
                t[0]=x
                t[1]=y
                #print()
                #aruco.drawAxis(frame, mtx, dist, rvec, tvec, 0.5) #Draw Axis
                #d = relativeAngle2D(firstRvec, firstTvec, rvec, tvec, mtx, dist)
                d =  errAngle(originAngle, curD) 
                if ids[i][0] == 11:
                    objj.append(Obj(ids[i][0],[1000*t[0][0],-1000*t[1][0],d],400,300)) 
                elif ids[i][0] == 1 or ids[i][0] == 2:
                    objj.append(Obj(ids[i][0],[1000*t[0][0],-1000*t[1][0],d],420,230))
                else:
                    objj.append(Obj(ids[i][0],[1000*t[0][0],-1000*t[1][0],d],240,170))
                
          
        
        ###### DRAW ID #####
        if t is not None:
            for i in range(len(objj)):                
                cv2.putText(frame, "ID:" + str(objj[i].ID) +  " x:" + str(int(objj[i].pos[0])) + " y:" + str(int(objj[i].pos[1])) + " d:" + str(int(objj[i].pos[2]*180/np.pi)), (0,40*(i+1)), font, 1, (0,255,0),2,cv2.LINE_AA)
            
            
    return frame, objj


while 1:
    ret, frame = cap.read()
    frame, objj = tracking(frame)
    cv2.imshow('frame',frame)
    for obj in objj:
        objs[obj.ID] = obj
            
    m.draw(objs)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
