# Usage example: python iou_eval.py test-0.txt ground-truth-0.txt
import sys
import numpy as np

# two assisting functions
def area(width, height):
    return width*height

# (xa1, ya1) is top left corner of box a
# (xa2, ya2) is bottom right corner of box a
# same for box b
def intersection_area(xa1, ya1, xa2, ya2, xb1, yb1, xb2, yb2):
    dx = min(xa2, xb2) - max(xa1, xb1)
    dy = min(ya2, yb2) - max(ya1, yb1)
    if (dx>=0) and (dy>=0):
        return dx*dy
    else:
        return 0

resultsFolder = sys.argv[1]
groundTruthFolder = sys.argv[2]

for fileNumber in range(0, len(os.listdir(resultsFolder))):

    # read input files and preallocate arrays
    prediction = np.loadtxt(resultsFolder + "/test-" + str(fileNumber) + ".txt",dtype=int,ndmin=2)
    npr = prediction.shape[0]
    
    groundtruth = np.loadtxt(groundTruthFolder + "/test-" + format(fileNumber, '03') + ".txt",dtype=int,ndmin=2)
    groundtruth = groundtruth[0]
    groundtruth = np.reshape(groundtruth, (len(groundtruth)/2, 2))
    
    ngt = groundtruth.shape[0]
    union_area = np.zeros((ngt, npr))
    intersect_area = np.zeros((ngt, npr))
    
#    if fileNumber == 1:
#        print groundtruth        
#        print prediction
    
    for g in range (0, ngt):
#    for g in groundtruth:
        # 1. assign ground truth bounding box corners' x and y coordinates to variables
        # 2. compute area of bounding box rectangle
        # Note this is the lazy version which assumes all ground truth rectangles are always the same shape (100x40)
        gtArea = 4000
        for p in range (0, npr):
#        for p in prediction:
            # 3. assign predicted bounding box corners' coordinates to variables
            # 4. compute area of predicted bounding box rectangle
            predictionArea = (prediction[p,2] - prediction[p,0]) * (prediction[p,3] - prediction[p,1])
            # 5. call intersection_area
            intersect_area[g,p] = intersection_area(groundtruth[g,0], groundtruth[g,1], groundtruth[g,0]+100, groundtruth[g,1]+40, prediction[p,0], prediction[p,1], prediction[p,2], prediction[p,3])
            # 6. compute the area of the union of the two bounding boxes
            union_area[g,p] = gtArea + predictionArea - intersect_area[g,p]

#            if fileNumber==1:
#                print predictionArea
#                print intersect_area
#                print union_area
                
    IOU = np.append(IOU, np.amax(intersect_area / union_area))
    
print IOU
print np.mean(IOU)
print np.std(IOU)