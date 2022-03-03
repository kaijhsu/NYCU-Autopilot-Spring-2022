import cv2
import numpy as np

def HistogramEqualization(imgPath,newImgPath):
    img = cv2.imread(imgPath)
    height, width, channels = img.shape
    img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    intensity = np.zeros((256), np.uint32)
    
    for y in range(height):
        for x in range(width):
            intensity[img[y,x,2]] += 1
            
    intensity_map = np.zeros((256), np.double)
    total = height*width
    for i in range(256):
        intensity_map[i] = 256*intensity[i]/total
        if i > 0:
            intensity_map[i] += intensity_map[i-1]
    intensity_map = np.round(intensity_map)
#     print(intensity_map)
    for y in range(height):
        for x in range(width):
            img[y,x,2] = min(255,intensity_map[img[y,x,2]])
    img = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
    cv2.imshow("result",img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    cv2.imwrite(newImgPath, img)
    return img
    
HistogramEqualization("kifune.jpg","kifune_out.jpg")