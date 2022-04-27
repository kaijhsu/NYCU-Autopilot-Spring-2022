import cv2
import numpy as np
cap = cv2.VideoCapture(0)
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

f = cv2.FileStorage("lab8\Calibration_50.xml", cv2.FILE_STORAGE_READ)
intrinsic = f.getNode("intrinsic").mat()
distortion = f.getNode("distortion").mat()
# print(f"intrinsic = {intrinsic}")
# print(f"distortion = {distortion}")
tvec2_record = []
while True:
    ret, img = cap.read()
    rects, weights = hog.detectMultiScale(img, winStride=(
        8, 8), scale=1.1, useMeanshiftGrouping=False)

    objp = np.zeros((3*3, 3), np.float32)
    objp[:, :2] = np.mgrid[0:101:50, 0: 201:100].T.reshape(-1, 2)
    # print(objp)
    # objp = np.zeros((9*6, 3), np.float32)
    # objp[:, :2] = np.mgrid[0:6*2.4:2.4, 0: 9*2.4:2.4].T.reshape(-1, 2)

    for rect in rects:
        imgpoints = []
        x = rect[0]
        y = rect[1]
        h = rect[2]
        w = rect[3]
        img = cv2.rectangle(
            img, (int(x), int(y)), (int(x+h), int(y+w)), (255, 0, 0), 3)
        # imgpoints = [[[x, y]], [[x+h, y+w]]]
        imgpoints.append([[x, y]])
        imgpoints.append([[x+h/2, y]])
        imgpoints.append([[x+h, y]])

        imgpoints.append([[x, y+w/2]])
        imgpoints.append([[x+h/2, y+w/2]])
        imgpoints.append([[x+h, y+w/2]])

        imgpoints.append([[x, y+w]])
        imgpoints.append([[x+h/2, y+w]])
        imgpoints.append([[x+h, y+w]])

        if imgpoints != []:
            imgpoints = np.array(imgpoints, np.float32)
            retval, rvec, tvec = cv2.solvePnP(
                objp, imgpoints, intrinsic, distortion)
            refine_z = 0.9596*int(tvec[2]) - 203.7037
            # cv2.putText(img, str(f"rvec = {np.round_(rvec)}"),
            #             (x-50, y+100), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
            cv2.putText(img, str(f"refine_z = {np.round_(refine_z)}"),
                        (x-50, y+50), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
            # print(f"rvec: {np.round_(rvec)}")
            # print(f"tvec: {np.round_(tvec)}")
            print(refine_z)
            # tvec2_record.append(int(tvec[2]))

    cv2.imshow("Image", img)
    cv2.waitKey(33)
    # print(tvec2_record)
