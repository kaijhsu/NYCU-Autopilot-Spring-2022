from cProfile import label
import cv2
import numpy as np
from numba import njit

class DisjSet:
    def __init__(self, n):
        self.rank = [1] * n
        self.parent = [i for i in range(n)]

    def find(self, x):
        if (self.parent[x] != x):
            self.parent[x] = self.find(self.parent[x])
        return self.parent[x]

    def union(self, x, y):
        xset = self.find(x)
        yset = self.find(y)
        if xset == yset:
            return
        if self.rank[xset] < self.rank[yset]:
            self.parent[xset] = yset
 
        elif self.rank[xset] > self.rank[yset]:
            self.parent[yset] = xset

        else:
            self.parent[yset] = xset
            self.rank[xset] = self.rank[xset] + 1
    
@njit
def inConnectedComponent(frame,labels):
    ret = list()
    height, width = frame.shape
    label_cnt = 1
    for y in range(height):
        for x in range(width):
            pixel = frame[y][x]
            label_x = x+1
            label_y = y+1
            if pixel == 0:
                labels[label_y, label_x] = 0
                continue
            
            upLabel = labels[label_y-1, label_x]
            leftLabel = labels[label_y, label_x-1]
            value = 0
            if(upLabel == 0 and leftLabel == 0):
                value = label_cnt
                label_cnt += 1
            elif(upLabel != 0 and leftLabel == 0):
                value = upLabel
            elif(upLabel == 0 and leftLabel != 0):
                value = leftLabel
            elif(upLabel == leftLabel):
                value = upLabel
            else:
                value = leftLabel
                ret.append((leftLabel, upLabel))
            labels[label_y][label_x] = value
    return ret

@njit
def afterConnectedComponent(labels, parents):
    height, width = labels.shape
    for y in range(height):
        for x in range(width):
            labels[y, x] = parents[labels[y, x]]
    return labels
            

def connectedComponent(frame):
    height, width = frame.shape
    
    #simplify algorithm, avoid edge condition
    labels = np.zeros((height+2,width+2), np.uint64)
    unionList = inConnectedComponent(frame, labels)
    disjoin = DisjSet(2000)
    for i, j in unionList:
        disjoin.union(i,j)
            
    labels = labels[1:height+1, 1:width+1]

    parents = list(range(2000));
    for i in range(2000):
        parents[i] = disjoin.find(i);
    labels = afterConnectedComponent(labels, parents)
    return labels

 
def connectedComponentTest():
    testcase = [[1,1,1,1,1,1], \
                [0,0,0,0,0,1], \
                [0,1,0,1,0,1], \
                [1,1,1,1,1,1], \
                [0,0,0,0,0,1], \
                [1,0,0,0,0,1]]
    arr = np.array(testcase)
    print(f"testcase\n", arr)
    ret = connectedComponent(arr)
    print(f"result\n", ret)
    scc = selectConnectedComponent(ret, 1)
    print(f"scc\n", scc)
    
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
   
    
# connectedComponentTest()
            

cap = cv2.VideoCapture('vtest.mp4')
if cap.isOpened() == False:
    print("video open failed!")

backSub = cv2.createBackgroundSubtractorMOG2()
    
cnt = 1
while True:
    cnt +=1;
    ret, frame = cap.read()
    if ret == False:
        break
    fgmask = backSub.apply(frame)
    
    shadowval = backSub.getShadowValue()
    ret, nmask = cv2.threshold(fgmask, shadowval, 255, cv2.THRESH_BINARY)
    labels = connectedComponent(nmask);
    # label_cnt, labels = cv2.connectedComponents(nmask);
    rects = selectConnectedComponent(labels,200)
    
    for key, value in rects.items():
        p1, p2 = (value['minX'],value['minY']), (value['maxX'], value['maxY'])
        cv2.rectangle(frame, p1, p2, 0x202F61, 2)
    

    cv2.imshow("frame", frame)
        
        
    cv2.waitKey(1)
    
    
cv2.destroyAllWindows()
print("end!")
