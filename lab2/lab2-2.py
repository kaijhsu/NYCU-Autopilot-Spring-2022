import cv2
import numpy as np

def compute(img, threshold):
    pass

def OtsuThreshold(imgPath,newImgPath):
    img = cv2.imread(imgPath)
    height, width, channels = img.shape
    
    intensity = np.zeros((256), np.uint32)
    for y in range(height):
        for x in range(width):
            intensity[img[y, x]] += 1
    nB = 0
    nO = width*height
    uB = 0
    uO = np.sum([i*intensity[i] for i in range(256)])/(width*height)
    
    bestThreshold = 0
    maxValue = 0
    
    for i in range(256):
        value = nB*nO*(uB-uO)**2
        nT = intensity[i]
        nB_new = nB + nT
        nO_new = nO - nT
        uB_new = (uB*nB+nT*i)/nB_new
        uO_new = (uO*nO-nT*i)/nO_new

        nB = nB_new
        nO = nO_new
        uB = uB_new
        uO = uO_new
        
        if value > maxValue:
            maxValue = value
            bestThreshold = i
    
    print(f"bestThreshols {bestThreshold}")
    
    for y in range(height):
        for x in range(width):
            if img[y,x,0] > bestThreshold:
                img[y,x] = 255, 255, 255
            else:
                img[y,x] = 0, 0, 0
    cv2.imshow("result",img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    cv2.imwrite(newImgPath, img)
    return img
    
OtsuThreshold("input.jpg","input_out.jpg")