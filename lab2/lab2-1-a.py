import cv2
import numpy as np

def HistogramEqualization(imgPath,newImgPath):
    img = cv2.imread(imgPath)
    height, width, channels = img.shape
    B = np.zeros((256), np.uint32)
    R = np.zeros((256), np.uint32)
    G = np.zeros((256), np.uint32)
    for y in range(height):
        for x in range(width):
            B[img[y,x,0]] += 1
            G[img[y,x,1]] += 1
            R[img[y,x,2]] += 1

    B_map = np.zeros((256), np.double)
    R_map = np.zeros((256), np.double)
    G_map = np.zeros((256), np.double)
    total = height*width
    for i in range(256):
        B_map[i] = 256*B[i]/total
        G_map[i] = 256*G[i]/total
        R_map[i] = 256*R[i]/total
        if i > 0:
            B_map[i] += B_map[i-1]
            G_map[i] += G_map[i-1]
            R_map[i] += R_map[i-1]
    B_map = np.round(B_map)
    G_map = np.round(G_map)
    R_map = np.round(R_map)
#     print(intensity_map)
    for y in range(height):
        for x in range(width):
            img[y,x,0] = min(255,B_map[img[y,x,0]])
            img[y,x,1] = min(255,G_map[img[y,x,1]])
            img[y,x,2] = min(255,R_map[img[y,x,2]])
    cv2.imshow("result",img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    cv2.imwrite(newImgPath, img)
    return img

HistogramEqualization("kifune.jpg","RGBkifune_out.jpg")
