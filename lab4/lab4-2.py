
import cv2
import numpy as np
from numba import njit

def warp(src, dst, mat):
    height, width = dst.shape[:2]

    for row in range(src.shape[0]):
        for col in range(src.shape[1]):
            p = np.array([[col],[row],[1.]], dtype=float)
            t = np.matmul(mat, p)
            t[0,0] /= t[2,0]
            t[1,0] /= t[2,0]
            t[2,0] /= t[2,0]

            if t[0,0] < 0 or t[0,0] >= width or t[1,0] < 0 or t[1,0] >= width:
                pass
            else:
                dst[int(t[1,0]), int(t[0,0])] = src[row, col]


camera = cv2.VideoCapture(-1)

img = cv2.imread('broadway.jpg')
ret, cap = camera.read()

# cap = cv2.imread('broadway.jpg')
img_y, img_x = img.shape[:2]
cap_y, cap_x, channel = cap.shape
cap_y -= 1
cap_x -= 1
# "x y" of image
# [430,205],[615,100],[430,430],[630,420]
img_corner = np.float32([[430,205],[615,100],[430,430],[630,420]])
cap_corner = np.float32([[0,0], [cap_x,0], [0,cap_y], [cap_x,cap_y]])

trans_mat = cv2.getPerspectiveTransform(cap_corner, img_corner)
# projected_cap = cv2.warpPerspective(cap, trans_mat, (img_x,img_y))
while True:
    ret, cap = camera.read()
    new_img = np.copy(img)
    warp(cap, new_img, trans_mat)
    cv2.imshow('img',new_img)
    cv2.waitKey(33)

# for y in range(img_y):
#     for x in range(img_x):
#         if(np.any(projected_cap[y, x])):
#             new_img[y,x] = projected_cap[y,x]


