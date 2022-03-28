from cv2 import VideoCapture
import tello
import cv2
import time
import numpy as np


def main():
    drone = tello.Tello('', 8889)
    time.sleep(10)
    patternSize = (9, 6)
    objpoints = []
    imgpoints = []
    while(True):
        frame = drone.read()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        # Lab4-1
        objp = np.zeros((9*6, 3), np.float32)  # create h*w (0,0,0)
        objp[:, :2] = np.mgrid[0:6, 0:9].T.reshape(-1, 2)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(frame, patternSize, None)
        if ret == True:
            objpoints.append(objp)
            cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                             (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1))
            imgpoints.append(corners)
            if len(imgpoints) == 5:
                break
        cv2.imshow("drone", frame)
        key = cv2.waitKey(1)
        if key != -1:
            drone.keyboard(key)
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None)
    f = cv2.FileStorage("Calibration.xml", cv2.FILE_STORAGE_WRITE)
    f.write("intrinsic", mtx)
    f.write("distortion", dist)
    f.release()


if __name__ == "__main__":
    main()