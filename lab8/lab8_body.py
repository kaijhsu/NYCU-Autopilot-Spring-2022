import cv2
import numpy as np
cap = cv2.VideoCapture(0)
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

f = cv2.FileStorage("Cablibration.xml", cv2.FILE_STORAGE_READ)
intrinsic = f.getNode("intrinsic").mat()
distortion = f.getNode("distortion").mat()
print(intrinsic)
print(distortion)
imgpoints = []
while True:
    ret, img = cap.read()
    rects, weights = hog.detectMultiScale(img, winStride=(
        6, 6), scale=1.1, useMeanshiftGrouping=False)
    # print(f"rects = {rects}")
    # print(f"weights = {weights}")
    for rect in rects:
        if rects != ():
            x = rect[0]
            y = rect[1]
            h = rect[2]
            w = rect[3]
            img = cv2.rectangle(
                img, (x, y), (x+h, y+w), (255, 0, 0), 3)
    # objp
    objp = np.zeros((9*6, 3), np.float32)
    objp[:, :2] = np.mgrid[0:6*2.4:2.4, 0: 9*2.4:2.4].T.reshape(-1, 2)
    # imgPoints
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(img, (9, 6), None)
    if ret == True:
        cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1),
                         (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1))
    imgpoints.append(corners)
    if len(imgpoints) == 5:
        break
    cv2.imshow("Image", img)
    cv2.waitKey(1)
# ret, intrinsic, distortion, rvecs, tvecs = cv2.calibrateCamera(objp, imgpoints, gray.shape[::-1], None, None)
retval, rvec, tvec = cv2.solvePnP(objp, imgpoints, intrinsic, distortion)
print(f"rvec: {rvec}")
print(f"tvec: {tvec}")
