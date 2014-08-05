from __future__ import print_function
import numpy
import json
import cv2
import time
from blob_finder import BlobFinder

def getJsonFrameToSigDict(fileName):
    with open(fileName,'r') as f:
        jsonData = json.load(f)
    frameToSig = {}
    for item in jsonData:
        seq = item['seq']
        sig = item['sync_signal'] 
        frameToSig[seq] = sig
    return frameToSig

def getAviFrameToSigDict(fileName, coord='x'):

    # Zeroing region  - for removing frame counter number from image
    zeroN, zeroM = 100, 100

    # Point tol
    ptTol = 4

    # Blob finder parameters
    threshold = 50 
    filterByArea = True 
    minArea = 20 
    maxArea = None
    blobFinder = BlobFinder(
            threshold=threshold,
            filterByArea=filterByArea,
            minArea=minArea,
            maxArea=maxArea
            )

    # Read frames from video and get list of blob centroid
    cap = cv2.VideoCapture(fileName)
    frmNum = 0
    numBlobTest = True
    frmToPtList = {}
    while (cap.isOpened()):
        ret, frame = cap.read()
        if not ret:
            break
        frmNum += 1
        print('processing frame: {0}'.format(frmNum))
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray[:zeroN,:zeroM] = numpy.zeros((zeroN,zeroM)) 
        blobList, blobImage = blobFinder.find(gray)
        ptList = []
        for blob in blobList:
            if coord == 'x':
                val = blob['centroidX']
            else:
                val = blob['centroidY']
            ptList.append(val)
        frmToPtList[frmNum] = ptList

        cv2.imshow('frame', blobImage)
        if cv2.waitKey(2) & 0xff == ord('q'): 
            break
        # -----------------------------------------------
        # Temp - for development, stop and examine frame 
        # -----------------------------------------------
        #if frmNum == 2086:
        #    ans = raw_input('paused')
    cap.release()
    cv2.destroyAllWindows()


    # Get unique set of point values - based
    ptSet = set()
    for frmNum, ptList in frmToPtList.iteritems():
        for val in ptList:
            found = False
            for pt in ptSet:
                if abs(pt - val) < ptTol:
                    found = True
            if not found:
                ptSet.add(val)

    print(ptSet)
    if len(ptSet) > 3:
        raise ValueError, 'more than three unique pts found'

    # Create pt to signal number dictionary
    ptList = list(ptSet)
    ptList.sort()
    ptToSigNum = dict([(x, ptList.index(x)) for x in ptList])

    # Create  frame number to signal dictionary
    frmToSig = {}
    for frmNum, ptList in  frmToPtList.iteritems():
        sig = [0,0,0]
        for x in ptList:
            closest = min([(abs(pt-x), sigNum) for pt,sigNum in ptToSigNum.iteritems()])
            ind = closest[1]
            sig[ind] = 1
        frmToSig[frmNum] = sig
    return frmToSig


# -----------------------------------------------------------------------------
if __name__ == '__main__':

    jsonFile = 'sleep_box_tracking_pts_logger.json'
    aviFile = 'sleep_box_image_stitched_labeled.avi'

    jsonFrameToSig = getJsonFrameToSigDict(jsonFile)
    aviFrameToSig = getAviFrameToSigDict(aviFile)

    test = True
    for frmNum, sigAvi in aviFrameToSig.iteritems():
        sigJson = jsonFrameToSig[frmNum]
        failStop = False
        if sigAvi != sigJson:
            test = False
            failStop = True
        fmtVals = frmNum, sigJson, sigAvi, sigAvi==sigJson
        print('frm: {0}, json: {1}, avi: {2}, test: {3}'.format(*fmtVals))
        if failStop:
            ans = raw_input('failStop')
    print()
    print('all test = ', test)












