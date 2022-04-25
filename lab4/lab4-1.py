import cv2
import numpy as np

cap = cv2.VideoCapture(0)  # device

patternSize = (9, 6)
objpoints = []
imgpoints = []

while(True):
    ret, frame = cap.read()
    if ret == False:
        print("Cannot read video")
        break
    objp = np.zeros((9*6, 3), np.float32)  # create h*w (0,0,0)

    objp[:, :2] = np.mgrid[0:6, 0:9].T.reshape(-1, 2)
    # print(objp)
    # print(objp[:, :2])
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(frame, patternSize, None)
    # print(corners)
    if ret == True:
        objpoints.append(objp)
        cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1),
                         (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1))
        print(corners)
        imgpoints.append(corners)
        if len(imgpoints) == 5:
            break

    cv2.imshow('frame', frame)
    cv2.waitKey(33)
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None)
f = cv2.FileStorage(".\Calibration.xml", cv2.FILE_STORAGE_WRITE)
f.write("intrinsic", mtx)
f.write("distortion", dist)
f.release()
