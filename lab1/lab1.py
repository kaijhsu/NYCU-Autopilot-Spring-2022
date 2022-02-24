import cv2
import numpy as np
image = cv2.imread("nctu_flag.jpg")
height, width, channels = image.shape
newimg = np.zeros((height,width,3),np.uint8)
# print(image[0,0])
for i in range(0,height):
    for j in range(0, width):
        B,G,R = image[i,j]
        GR = (int(B)+int(G)+int(R))/3
        if B > 30 and B*0.9>G and B*0.9 >R:
            newimg[i,j] = image[i,j]
        else:
            newimg[i,j] = GR,GR,GR

cv2.imwrite("output.jpg", newimg)
cv2.imshow("AAA", newimg)
cv2.waitKey(0)
cv2.destroyAllWindows()
