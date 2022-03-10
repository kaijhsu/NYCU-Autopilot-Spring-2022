import cv2
import numpy as np
from numba import njit     

@njit
def connectedComponent(frame):
    height, width = frame.shape
    
    #simplify algorithm, avoid edge condition
    labels = np.zeros((height+2,width+2), np.uint32)
    label_cnt = 1
    
    parent = dict()
    parent[0] = 0
    parent[1] = 1
        
    for y in range(height):
        for x in range(width):
            pixel = frame[y][x]
            label_x = x+1
            label_y = y+1
            if pixel == 0:
                labels[label_y, label_x] = 0
                continue
            
            upLabel = labels[y-1, x]
            leftLabel = labels[y, x-1]
            value = 0
            if(upLabel == 0 and leftLabel == 0):
                value = label_cnt
                label_cnt += 1
                parent[label_cnt] = label_cnt
            elif(upLabel != 0 and leftLabel == 0):
                value = upLabel
            elif(upLabel == 0 and leftLabel != 0):
                value = leftLabel
            elif(upLabel == leftLabel):
                value = upLabel
            else:
                value = upLabel
                parent[leftLabel] = parent[upLabel]
            labels[label_y][label_x] = value
    
    for i in range(label_cnt):
        p = parent[i]
        pp = parent[parent[i]]
        while p != pp:
            p = pp
            pp = parent[pp]
        parent[i] = p
    for i in range(label_cnt):
        p = parent[i]
        pp = parent[parent[i]]
        if p != pp:
            print(p,pp)
    
    labels = labels[1:height+1, 1:width+1]
    for y in range(height):
        for x in range(width):
            labels[y, x] = parent[labels[y,x]]
    return labels

@njit
def selectConnectedComponent(keymap,T):
    areaDict = {}
    minmaxDict = {}
    height, width = keymap.shape
    
    for y in range(height):
        for x in range(width):
            pixel = keymap[y][x]
            
            if(pixel in areaDict):
                areaDict[pixel] += 1
            else:
                areaDict[pixel] = 1
            if(pixel in minmaxDict):
                value = minmaxDict[pixel]
                if(x < value["minX"]):
                    minmaxDict[pixel]["minX"] = x
                if(x > value["maxX"]):
                    minmaxDict[pixel]["maxX"] = x
                if(y < value["minY"]):
                    minmaxDict[pixel]["minY"] = y
                if(y > value["maxY"]):
                    minmaxDict[pixel]["maxY"] = y

            else:
                minmaxDict[pixel] = {"minX": x,
                                     "maxX": x,
                                     "minY": y,
                                     "maxY": y
                                    }
            

    del areaDict[0]
    del minmaxDict[0]

    for dictkey in list(areaDict.keys()):
        value = areaDict[dictkey]
        if(value < T):
            del areaDict[dictkey]
            del minmaxDict[dictkey]
    return minmaxDict
    
    
            

cap = cv2.VideoCapture('vtest.mp4')
if cap.isOpened() == False:
    print("video open failed!")

backSub = cv2.createBackgroundSubtractorMOG2()
    
while True:
    ret, frame = cap.read()
    if ret == False:
        break
    fgmask = backSub.apply(frame)
    
    shadowval = backSub.getShadowValue()
    ret, nmask = cv2.threshold(fgmask, shadowval, 255, cv2.THRESH_BINARY)
    labels = connectedComponent(nmask);
    rects = selectConnectedComponent(labels,100)

    for key, value in rects.items():
        p1, p2 = (value['minX'],value['minY']), (value['maxX'], value['maxY'])
        cv2.rectangle(frame, p1, p2, 0x202F61, 2)
    
    
    cv2.imshow("frame", frame)
    cv2.waitKey(1)
    
    
cv2.destroyAllWindows()
print("end!")
