import numpy as np
import cv2
import yaml
import keyboard
import time

# Press space bar to capture image


# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*9,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

cap = cv2.VideoCapture(0) #cv2.CAP_DSHOW + 0
a = cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)
b = cap.set(cv2.CAP_PROP_EXPOSURE, -4)
#print(a,b)
cap.set(3,800)
cap.set(4,600)

found = 0
getImg = False
while(found < 10):  # Here, 10 can be changed to whatever number you like to choose
    ret, img = cap.read() # Capture frame-by-frame
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (9,6),None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        # Draw and display the corners
        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        img = cv2.drawChessboardCorners(img, (9,6), corners2, ret)
        if getImg:            
            objpoints.append(objp)   # Certainly, every loop objp is the same, in 3D.            
            imgpoints.append(corners2)
            print("Get Image #:",found)
            found += 1
            getImg = False
    
    if keyboard.is_pressed(' '):
        getImg = True        
        time.sleep(0.2)
    cv2.imshow('img', img)
    cv2.waitKey(10)

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
print(mtx)
print(dist)
# It's very important to transform the matrix to list.

data = {'camera_matrix': np.asarray(mtx).tolist(), 'dist_coeff': np.asarray(dist).tolist()}

with open("calibration.yaml", "w") as f:
    yaml.dump(data, f)
    