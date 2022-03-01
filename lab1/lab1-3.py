#!/usr/bin/env python
# coding: utf-8

# In[ ]:


import cv2
import numpy as np
import math

def scale(img, height, width, n):
    newimg = np.zeros((height*n, width*n, 3), np.uint8)
    for i in range(height-1):
        if i%10 == 0:
            print(i)
        for j in range(width-1):
            for x in range(n):
                for y in range(n):
                    Lrange = x
                    Rrange = n-x
                    topCenter = (float(Rrange/n)*img[i,j]+float(Lrange/n)*img[i,j+1])
                    downCenter = (float(Rrange/n)*img[i+1,j]+float(Lrange/n)*img[i+1,j+1])
                    Trange = y
                    Drange = n-y
                    Center = (topCenter*Drange+downCenter*Trange)/n
                    newimg[n*i+y,n*j+x] = np.round(Center)
#                     print(f"\tx {x}, y {y}")
#                     print(f"\tTop: left {img[i,j]}, right {img[i,j+1]}, center {topCenter}")
#                     print(f"\tDow: left {img[i+1,j]}, right {img[i+1][j+1]}, center {downCenter}")
#                     print(f"\tCenter {Center}, newimg {newimg[n*i+x, n*j+y]}")
                    
    return newimg
            


image = cv2.imread("lisa.jpeg")
height, width, channels = image.shape


newimg = scale(image,height,width,3)

cv2.imwrite("output3.jpg", newimg)
cv2.imshow("new img", newimg)
cv2.waitKey(0)
cv2.destroyAllWindows()



