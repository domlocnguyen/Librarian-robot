import numpy
import cv2
import cv2.aruco as aruco
import os
import pickle
from picamera import PiCamera,Color
from imutils.video import VideoStream
import imutils
import argparse


class MyAruco:
    def __init__ (self, camCal_filename='./calibration.pckl'):

        self.ARUCO_PARAMETERS = aruco.DetectorParameters_create()
        self.ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_5X5_100)
        self.vs = VideoStream()
        self.vs.start()
#        self.board =aruco.GridBoard_create(
#            markersX=2,
#            markersY=2,
#            markerLength=0.009,
#            markerSeparation=0.01,
#            dictionary=self.ARUCO_DICT)
        self.rvecs= None
        self.tvecs = None

        if not os.path.exists(camCal_filename):
            print("You need to calibrate the camera you'll be using. See calibration project directory for details.")
            exit()
        else:
            f = open(camCal_filename, 'rb')
            (self.cameraMatrix, self.distCoeffs, _, _) = pickle.load(f)
            #print(cameraMatrix, distCoeffs);
            f.close()
            if self.cameraMatrix is None or self.distCoeffs is None:
                print("Calibration issue. Remove ./calibration.pckl and recalibrate your camera with CalibrateCamera.py.")
                exit()


    def detectAndGetTargetDist (self,targetID, poseEst = 0):
        
        found = False
        self.rvecs= None
        self.tvecs = None
        dis = None
        # Capturing each frame of our video stream
        QueryImg = self.vs.read()
        
        # grayscale image
        gray = cv2.cvtColor(QueryImg, cv2.COLOR_BGR2GRAY)
            
        # Detect Aruco markers
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.ARUCO_DICT, parameters=self.ARUCO_PARAMETERS)
        #print(ids)
                
        # Refine detected markers
        # Eliminates markers not part of our board, adds missing markers to the board
#        corners, ids, rejectedImgPoints, recoveredIds = aruco.refineDetectedMarkers(
#            image = gray,board = self.board, detectedCorners = corners, detectedIds = ids, rejectedCorners = rejectedImgPoints,
#            cameraMatrix = self.cameraMatrix,distCoeffs = self.distCoeffs)
        
        # Make sure all 5 markers were detected before printing them out
        d=1000
        if ids is not None: 
            # Output all of the markers detected in our image in green
            #QueryImg = aruco.drawDetectedMarkers(QueryImg, corners, ids, borderColor=(0, 255, 0))
            if targetID in ids:
                found = True
                targetIndex =0
                while (1):
                    if ids[targetIndex] ==targetID:
                        break
                    targetIndex=targetIndex+1;
                targetCorners = numpy.array(corners)[targetIndex,:,:]
#                c = targetCorners.reshape(4,2)
#                mw = c[1][0] + c[2][0]
#                dis = (mw/2 - 320)*12.5/mw
#                if poseEst:                            
                rvec, tvec, __ = aruco.estimatePoseSingleMarkers(targetCorners, 0.0125, self.cameraMatrix, self.distCoeffs)                    
                #QueryImg = aruco.drawAxis(QueryImg, self.cameraMatrix, self.distCoeffs, self.rvecs, self.tvecs, 1)
                tvec= tvec[0][0]
                
                d = (-tvec[0] - tvec[2]/3.55)*1000
                #print('rotation and translation vector', self.rvecs, self.tvecs)                   
        #cv2.imshow('frame',QueryImg)             
        return found, d
    def __del__(self):
        self.vs.stop()
#ar = MyAruco()
#while 1:
#    f, dis, QueryImg = ar.detectAndGetTargetDist(10,1)
#    cv2.imshow('frame',QueryImg)
#    print(f,dis)
#    cv2.waitKey(1)
        
