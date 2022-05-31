import cv2
import numpy as np
import matplotlib.pyplot as plt


img1=cv2.imread("pic01.jpg")
img2=cv2.imread("pic03.jpg")
# img1 =cv2.resize(img1, (640, 480))
# img2 =cv2.resize(img2, (640, 480))
gray1=cv2.cvtColor(img1,cv2.COLOR_BGR2GRAY)
gray2=cv2.cvtColor(img2,cv2.COLOR_BGR2GRAY)
# detector=cv2.xfeatures2d.SIFT_create(contrastThreshold = 0.15)
# detector=cv2.xfeatures2d.SURF_create(11000)
detector = cv2.ORB_create()
kp1,des1=detector.detectAndCompute(gray1,None)
kp2,des2=detector.detectAndCompute(gray2,None)
# img = np.array([])
matcher = cv2.DescriptorMatcher_create("BruteForce")
matches = matcher.match(des1, des2, None)
img1=cv2.drawKeypoints(img1,kp1,None,(0,255,0),4)
img2=cv2.drawKeypoints(img2,kp2,None,(0,255,0),4)
img = cv2.drawMatches(img1, kp1, img2, kp2, matches, None)
cv2.namedWindow("result",0)
cv2.resizeWindow("result", 1600, 900)
cv2.imshow("result",img)
cv2.imwrite("ORB2.png", img)
cv2.waitKey(0)
cv2.destroyAllWindows()
