import cv2
from cv2 import UMat
import dlib
import numpy as np


cap = cv2.VideoCapture(0)
detector = dlib.get_frontal_face_detector()
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

f = cv2.FileStorage("../Calibration_KJ_Linux.xml", cv2.FILE_STORAGE_READ)
intrinsic = f.getNode("intrinsic").mat()
distortion = f.getNode("distortion").mat()

objectPoints_b = np.zeros((3*3, 3), np.float32)
objectPoints_b[:, :2] =  np.mgrid[0:101:50, 0: 201:100].T.reshape(-1, 2)


def estimateFaceDistance(width):
    # distance = width
    # return distance
    ...




def main():
    while True:
        ret, frame = cap.read()
        if ret != True :
            print("can't read!")
            exit(1)

        #detect face
        face_rects = detector(frame, 0)
        for i, d in enumerate(face_rects):
            x1 = d.left()
            y1 = d.top()
            x2 = d.right()
            y2 = d.bottom()
            frame = cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 3)
            # print(f"{x2-x1} {y2-y1}")
            objectPoints = np.array([ [ 0,  0, 0],
                                      [ 0,  7.5, 0],
                                      [ 0, 15, 0],
                                      [ 7.5,  0, 0],
                                      [ 7.5,  7.5, 0],
                                      [ 7.5, 15, 0],
                                      [15,  0, 0],
                                      [15,  7.5, 0],
                                      [15, 15, 0]], dtype=float)
            imagePoints  = np.array([[y1, x1],
                                     [y1, (x1+x2)/2],
                                     [y1, x2],
                                     [(y1+y2)/2, x1],
                                     [(y1+y2)/2, (x1+x2)/2],
                                     [(y1+y2)/2, x2],
                                     [y2, x1],
                                     [y2, (x1+x2)/2],
                                     [y2, x2]]).round()
            # print(imagePoints)
            retval, rvec, tvec = cv2.solvePnP(objectPoints, imagePoints, intrinsic, distortion)
            distance = tvec[2]/2
            frame = cv2.putText(frame, f"{distance}", (x1, y1), cv2.FONT_HERSHEY_PLAIN, 1.5, (0, 0, 255), 1, cv2.LINE_AA)
            # rvec, tvec = estimateDistance(test, imagePoints)

        #detect body
        rects, weights = hog.detectMultiScale(frame, winStride=(8, 8), scale=1.05, useMeanshiftGrouping = False)
        for rect in rects:
            x1 = rect[0]
            y1 = rect[1]
            x2 = x1 + rect[2]
            y2 = x2 + rect[3]
            frame = cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)
            imagePoints_b  = np.array([[y1, x1],
                                     [y1, (x1+x2)/2],
                                     [y1, x2],
                                     [(y1+y2)/2, x1],
                                     [(y1+y2)/2, (x1+x2)/2],
                                     [(y1+y2)/2, x2],
                                     [y2, x1],
                                     [y2, (x1+x2)/2],
                                     [y2, x2]]).round()
            # print(imagePoints)
            retval, rvec, tvec = cv2.solvePnP(objectPoints_b, imagePoints_b, intrinsic, distortion)
            distance = tvec[2]/2
            distance = int(distance)
            frame = cv2.putText(frame, f"{distance}", (x1, y1), cv2.FONT_HERSHEY_PLAIN, 1.5, (0, 0, 255), 1, cv2.LINE_AA)

        cv2.imshow("cap", frame)
        key = cv2.waitKey(5)
        if key != -1:
            cv2.destroyAllWindows()
            break

main()

# objectPoints = np.array([[ [0], [0], [0]],
#                                      [ [0], [10], [0]],
#                                      [ [0], [20], [0]],
#                                      [[10],  [0], [0]],
#                                      [[10], [10], [0]],
#                                      [[10], [20], [0]],
#                                      [[20],  [0], [0]],
#                                      [[20], [10], [0]],
#                                      [[20], [20], [0]]], dtype=float)
#             print(objectPoints.shape)
#             objectPoints = cv2.cvtColor(objectPoints, cv2.COLOR_GRAY2BGR)
#             imagePoints  = np.array([[[y1], [x1]],
#                                      [[y1], [(x1+x2)/2]],
#                                      [[y1], [x2]],
#                                      [[(y1+y2)/2], [x1]],
#                                      [[(y1+y2)/2], [(x1+x2)/2]],
#                                      [[(y1+y2)/2], [x2]],
#                                      [[y2], [x1]],
#                                      [[y2], [(x1+x2)/2]],
#                                      [[y2], [x2]]]).round()
#             imagePoints = cv2.cvtColor(imagePoints, cv2.COLOR_GRAY2BGR)
