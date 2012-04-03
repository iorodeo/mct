from __future__ import print_function
import roslib
roslib.load_manifest('mct_blob_finder')
import rospy
import cv
import cv2
import cvblob
import math
from cv_bridge.cv_bridge import CvBridge 

class BlobFinder(object):
    """
    A simple blob finder based on the cvBlob library.  Thresholds the image and filters
    the blobs by area if requested.
    """

    def __init__(self,threshold=200, filter_by_area=False, min_area=0, max_area=200):
        self.threshold = threshold
        self.filter_by_area = filter_by_area 
        self.min_area = min_area 
        self.max_area = max_area 
        self.bridge = CvBridge()
        self.blob_mask  = cvblob.CV_BLOB_RENDER_CENTROID 
        self.blob_mask |= cvblob.CV_BLOB_RENDER_COLOR 
        self.blobs_image = None
        self.label_image = None
        self.thresh_image = None

    def findBlobs(self, data, create_image=True):
        """
        Finds blobs in rosimage data. Returns 

        blobs - stucture containting data for all blobs found in the image which aren't
                filtered out.
        blobs_image - an image with the blobs and their contours.
        """
        # Convert rosimage to opencv image.
        cv_image = self.bridge.imgmsg_to_cv(data,desired_encoding="passthrough")
        raw_image = cv.GetImage(cv_image)

        if self.thresh_image is None:
            self.thresh_image = cv.CreateImage(cv.GetSize(raw_image),raw_image.depth, raw_image.channels)

        # Threshold image 
        cv.Threshold(raw_image, self.thresh_image, self.threshold, 255, cv.CV_THRESH_BINARY)

        ### Erode and dilate - problematic when tracking points are close together
        #cv.Dilate(self.thresh_image, self.thresh_image, None, 1)
        #cv.Erode(self.thresh_image, self.thresh_image, None, 1)

        # Find contours
        storage = cv.CreateMemStorage(0)
        contours = cv.FindContours(self.thresh_image,  storage, cv.CV_RETR_CCOMP, cv.CV_CHAIN_APPROX_SIMPLE)

        # Find blob data
        blobs_list = []
        blobs_contours = []

        while contours:

            blob_ok = True

            # Get area and apply area filter  
            area = cv.ContourArea(contours)
            if self.filter_by_area:
                if area < self.min_area or area > self.max_area or area <= 0.0:
                    blob_ok = False

            # Get centroid
            moments = cv.Moments(contours)
            if moments.m00 > 0 and blob_ok:
                centroid_x = moments.m10/moments.m00
                centroid_y = moments.m01/moments.m00
            else:
                blob_ok = False
                centroid_x = 0.0
                centroid_y = 0.0
            
            # Get bounding rectangle
            if blob_ok:
                bound_rect = cv.BoundingRect(list(contours))
                min_x = bound_rect[0]
                min_y = bound_rect[1]
                max_x = bound_rect[0] + bound_rect[2] 
                max_y = bound_rect[1] + bound_rect[3] 
            else:
                min_x = 0.0 
                min_y = 0.0
                max_x = 0.0
                max_y = 0.0

            # Create blob dictionary
            blob = {
                    'centroid_x' : centroid_x,
                    'centroid_y' : centroid_y,
                    'min_x'      : min_x,
                    'max_x'      : max_x,
                    'min_y'      : min_y,
                    'max_y'      : max_y,
                    'area'       : area,
                    'angle'      : 0.0,  # kept this for compatibility with cvblob version
                    }

            # If blob is OK add to list of blobs
            if blob_ok:
                blobs_list.append(blob)
                blobs_contours.append([(x,y) for x,y in contours])

            contours = contours.h_next()

        if not create_image:
            return blobs_list
        else:
            if self.blobs_image is None:
                self.blobs_image = cv.CreateImage(cv.GetSize(raw_image), cv.IPL_DEPTH_8U, 3)
            cv.CvtColor(raw_image,self.blobs_image,cv.CV_GRAY2BGR)

            # Draw blobs
            for i, cdata in enumerate(blobs_contours):
                p0_list = cdata
                p1_list = cdata[1:] + [cdata[0]]
                for p0,p1 in zip(p0_list,p1_list):
                    cv.Line(self.blobs_image, p0, p1,(0,0,255),2)
            #blobs_rosimage = self.bridge.cv_to_imgmsg(self.blobs_image,encoding="passthrough")
            #return blobs_list, blobs_rosimage
            return blobs_list, self.blobs_image


    def findBlobsCvBlob(self,data, create_image=True):
        """
        Old version using cvblob library - has a memory leak somewhere (in the C++ library or 
        python bindings) which makes is unusable
        

        Finds blobs in rosimage data. Returns 

        blobs - stucture containting data for all blobs found in the image which aren't
                filtered out.
        blobs_image - an image with the blobs and there centroids superimposed on the image.
        """
        # Convert rosimage to opencv image.
        cv_image = self.bridge.imgmsg_to_cv(data,desired_encoding="passthrough")
        raw_image = cv.GetImage(cv_image)

        if self.thresh_image is None:
            if create_image:
                self.thresh_image = cv.CreateImage(cv.GetSize(raw_image),raw_image.depth, raw_image.channels)
            else:
                self.thresh_image = raw_image

        # Threshold image 
        cv.Threshold(raw_image, self.thresh_image, self.threshold, 255, cv.CV_THRESH_BINARY)

        if self.label_image is None:
            self.label_image = cv.CreateImage(cv.GetSize(raw_image), cvblob.IPL_DEPTH_LABEL, 1)

        # Find blobs
        blobs = cvblob.Blobs()
        result = cvblob.Label(self.thresh_image, self.label_image, blobs)

        # Filter blobs by area
        if self.filter_by_area:
            cvblob.FilterByArea(blobs,self.min_area,self.max_area)

        # Convert blobs data structure to dictionary
        blobs_list = []
        for k in blobs:
            blob_dict = {}
            centroid = cvblob.Centroid(blobs[k])
            blob_dict['centroid_x'] = centroid[0]
            blob_dict['centroid_y'] = centroid[1]
            blob_dict['angle'] = cvblob.Angle(blobs[k])
            blob_dict['area'] = blobs[k].area
            blob_dict['min_x'] = blobs[k].minx
            blob_dict['max_x'] = blobs[k].maxx
            blob_dict['min_y'] = blobs[k].miny
            blob_dict['max_y'] = blobs[k].maxy
            blobs_list.append(blob_dict)

        if not create_image:
            return blobs_list
        else:
            # Render blobs on image
            if self.blobs_image is None:
                self.blobs_image = cv.CreateImage(cv.GetSize(raw_image), cv.IPL_DEPTH_8U, 3)
            cv.CvtColor(raw_image, self.blobs_image,cv.CV_GRAY2BGR)
            cvblob.RenderBlobs(self.label_image, blobs, raw_image, self.blobs_image, self.blob_mask, 1.0)
            blobs_rosimage = self.bridge.cv_to_imgmsg(self.blobs_image,encoding="passthrough")
            return blobs_list, blobs_rosimage
