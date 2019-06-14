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

import threading
import websocket
try:
    import thread
except ImportError:
    import _thread as thread
import time

DRAW_MAP = False
m = Map([-1000,3500],[-2000,2000])
markerLength = 0.117
objs = []

cap = cv2.VideoCapture(0) #cv2.CAP_DSHOW + 0
a = cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)
b = cap.set(cv2.CAP_PROP_EXPOSURE, -6)
cap.set(3,800)
cap.set(4,600)

# Read cabliration file
with open('calibration.yaml') as f:
    loadeddict = yaml.load(f)

mtx = np.array(loadeddict.get('camera_matrix'))
dist = np.array(loadeddict.get('dist_coeff'))
print(mtx)
print(dist)



def corners2Angle(corners):
    corners = corners.reshape(4,2)
    centre = corners[0] + corners[1] + corners[2] + corners[3]
    centre[:] = [int(x / 4) for x in centre]
    d = math.atan2(corners[1][1]-corners[0][1],corners[1][0]-corners[0][0])
    orientedPoint = tuple([int(50*math.cos(d) + centre[0]), int(50*math.sin(d) + centre[1])])
    return d, tuple(centre), orientedPoint

def errAngle(ref,cur):
    if abs(cur - ref) > np.pi:
        if (cur - ref) > 0:
            return (cur - ref) - 2*np.pi
        else:
            return (cur - ref) + 2*np.pi
    else:
        return (cur - ref)
    
originID = 0
originAngle = None
firstRvec = None
firstTvec = None


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
            rvec, tvec,_ = aruco.estimatePoseSingleMarkers(corners[i], markerLength, mtx, dist) #Estimate pose of each marker and return the values rvet and tvec---different from camera coefficients
            if (ids[i] == originID and firstTvec is None):
                firstRvec = rvec
                originAngle,centre,orient_centre = corners2Angle(corners[i])                
                firstTvec = tvec     
                
                #aruco.drawAxis(frame, mtx, dist, rvec, tvec, 0.5) #Draw Axis
                cv2.line(frame,centre,orient_centre,(255,0,255),4)
                c = corners[i].reshape(4,2)
                cv2.line(frame,tuple(c[0]),tuple(c[1]),(0,255,0),1)
                cv2.line(frame,tuple(c[1]),tuple(c[2]),(0,255,0),1)
                cv2.line(frame,tuple(c[2]),tuple(c[3]),(0,255,0),1)
                cv2.line(frame,tuple(c[3]),tuple(c[0]),(0,255,0),1)
                cv2.putText(frame, "Origin angle: " + str(int(originAngle*180/np.pi)), (0,575), font, 1, (0,0,255), 2, cv2.LINE_AA)
            elif (firstRvec is not None and ids[i]<12 and ids[i] != originID):
                curD,centre,orient_centre = corners2Angle(corners[i])
                cv2.line(frame,centre,orient_centre,(255,0,255),4)
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
                d = - errAngle(originAngle, curD)                
                if ids[i][0] == 11:
                    objj.append(Obj(ids[i][0],[1000*t[0][0],-1000*t[1][0],d],400,300)) 
                elif ids[i][0] == 1 or ids[i][0] == 2:
                    objj.append(Obj(ids[i][0],[1000*t[0][0],-1000*t[1][0],d],420,250))
                else:
                    objj.append(Obj(ids[i][0],[1000*t[0][0],-1000*t[1][0],d],240,170))
          
        
        ###### DRAW ID #####
        if t is not None:
            for i in range(len(objj)):                
                cv2.putText(frame, "ID:" + str(objj[i].ID) +  " x:" + str(int(objj[i].pos[0])) + " y:" + str(int(objj[i].pos[1])) + " d:" + str(int(objj[i].pos[2]*180/np.pi)), (0,40*(i+1)), font, 1, (0,255,0),2,cv2.LINE_AA)
            
            
    return frame, objj

# Calculate stable positons for initial map
from statistics import  mean
def mf(List): 
    return max(set(List), key = List.count)

def getInitMap(nItem=2):
    store = []
    n = 0
    while n<10:
        ret, frame = cap.read()
        frame, objj = tracking(frame)
        cv2.imshow('frame',frame)
        if (len(objj) == nItem):
            n +=1
            for obj in objj:
                store.append(obj)
        cv2.waitKey(1)
    
    dic = {}
    for s in store:
        if s.ID in dic.keys(): 
            dic[s.ID].append(s)
        else:
            dic[s.ID] = []
            dic[s.ID].append(s)
    result = {}
    for k in dic.keys():
        x=[]
        y=[]
        d=[]
        for obj in dic[k]:
            x.append(obj.pos[0])
            y.append(obj.pos[1])
            d.append(obj.pos[2])
       
        result[k] = Obj(k,[mean(x),mean(y),mf(d)],dic[k][0].length,dic[k][0].depth)
    return result



def on_message(ws, message):
    print(message)

def on_error(ws, error):
    print(error)

def on_close(ws):
    print("### closed ###")
    cap.release()
    cv2.destroyAllWindows()
    
def on_open(ws):
    def run(*args):
        init = getInitMap(3)        
        for k in init.keys():
            ws.send("&I" + "," + str(int(k)) + "," + str(int(init[k].pos[0])) + "," + str(int(init[k].pos[1])) + "," + str(init[k].pos[2]) + "," + str(int(init[k].length)) + "," + str(int(init[k].depth)))
            print("&I" + "," + str(int(k)) + "," + str(int(init[k].pos[0])) + "," + str(int(init[k].pos[1])) + "," + str(init[k].pos[2]) + "," + str(int(init[k].length)) + "," + str(int(init[k].depth)))
            time.sleep(0.1)
        if DRAW_MAP:
            init[12] = Obj(12,goalFromShelf(init[1],init[1].length/2 + 500),400,300)
            print("Goal: ", init[2].pos)
            m.draw(init)
              
        
        rMap = init
        while 1:
            ret, frame = cap.read()
            frame, objj = tracking(frame)
            cv2.imshow('frame',frame)
            for obj in objj:
                if obj.ID == 11:
                    ws.send("&R" + "," + str(int(obj.pos[0])) + "," + str(int(obj.pos[1])) + "," + str(obj.pos[2]))
                    #print("&R" + "," + str(int(obj.pos[0])) + "," + str(int(obj.pos[1])) + "," + str(obj.pos[2]))
                    rMap[obj.ID].pos = obj.pos
                    break
            if DRAW_MAP:
                m.draw(rMap)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            
    thread.start_new_thread(run, ())

websocket.enableTrace(False)
ws = websocket.WebSocketApp("ws://192.168.0.36:8000/",
                                on_message = on_message,
                                on_error = on_error,
                                on_close = on_close)
ws.on_open = on_open
ws.run_forever()