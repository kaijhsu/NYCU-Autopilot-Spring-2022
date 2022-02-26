#!/usr/bin/env python
# coding: utf-8

# In[ ]:


import cv2
import numpy as np
import math

def scale(img, height, width, n):
    newimg = np.zeros((height*n, width*n, 3), np.uint8)
    for i in range(height-1):
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
                    newimg[n*i+x,n*j+y] = Center
    return newimg
            


image = cv2.imread("lisa.jpeg")
height, width, channels = image.shape


newimg = scale(image,height,width,2)

cv2.imwrite("output3.jpg", newimg)
cv2.imshow("new img", newimg)
cv2.waitKey(0)
cv2.destroyAllWindows()


# In[ ]:




