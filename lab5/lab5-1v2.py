import cv2
import numpy as np
import cv2.aruco

cap = cv2.VideoCapture(0)  # device

patternSize = (9, 6)
objpoints = []
imgpoints = []
dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters_create()
# while(True):
#     ret, frame = cap.read()
#     if ret == False:
#         print("Cannot read video")
#         break
#     objp = np.zeros((9*6, 3), np.float32)  # create h*w (0,0,0)
#     objp[:, :2] = np.mgrid[0:6, 0:9].T.reshape(-1, 2)
#     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#     ret, corners = cv2.findChessboardCorners(frame, patternSize, None)
#     if ret == True:
#         objpoints.append(objp)
#         cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
#                          (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1))
#         imgpoints.append(corners)
#         if len(imgpoints) == 5:
#             break
#
#     cv2.imshow('frame', frame)
#     cv2.waitKey(33)
# ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
#     objpoints, imgpoints, gray.shape[::-1], None, None)
# f = cv2.FileStorage("Calibration.xml", cv2.FILE_STORAGE_WRITE)
# f.write("intrinsic", mtx)
# f.write("distortion", dist)
# f.release()

f = cv2.FileStorage("Calibration.xml", cv2.FILE_STORAGE_READ)
mtx = f.getNode("intrinsic").mat()
dist = f.getNode("distortion").mat()
f.release()
while(True):
    ret, frame = cap.read()
    if ret == False:
        print("Cannot read video")
        break
    markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)
    frame = cv2.aruco.drawDetectedMarkers(frame,markerCorners, markerIds)
    rvec, tvec, _objPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorners,15, mtx, dist)
    if rvec is None:
        cv2.imshow('frame', frame)
        cv2.waitKey(33)
        continue
    frame = cv2.aruco.drawAxis(frame, mtx,dist, rvec, tvec, 0.5)
    cv2.imshow('frame', frame)
    cv2.waitKey(33)
