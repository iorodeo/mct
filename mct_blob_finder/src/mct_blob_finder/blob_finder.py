from __future__ import print_function
import roslib
roslib.load_manifest('mct_blob_finder')
import rospy
import cv
import cvblob
from cv_bridge.cv_bridge import CvBridge 

class BlobFinder(object):
    """
    A simple blob finder based on the cvBlob library.  Thresholds the image and filters
    the blobs by area if requested.
    """

    def __init__(self,threshold=100,filter_by_area=False,min_area=0, max_area=200):
        self.threshold = threshold
        self.filter_by_area = filter_by_area 
        self.min_area = min_area 
        self.max_area = max_area 
        self.bridge = CvBridge()
        self.blob_mask  = cvblob.CV_BLOB_RENDER_CENTROID 
        self.blob_mask |= cvblob.CV_BLOB_RENDER_COLOR 
        self.blobs_image = None

    def findBlobs(self,data, create_image=True):
        """
        Finds blobs in rosimage data. Returns 

        blobs - stucture containting data for all blobs found in the image which aren't
                filtered out.
        blobs_image - an image with the blobs and there centroids superimposed on the image.
        """
        # Convert rosimage to opencv image.
        cv_image = self.bridge.imgmsg_to_cv(data,desired_encoding="passthrough")
        raw_image = cv.GetImage(cv_image)
        thresh_image = cv.CreateImage(cv.GetSize(raw_image),raw_image.depth, raw_image.channels)

        # Find blobs in image
        cv.Threshold(raw_image, thresh_image, self.threshold, 255, cv.CV_THRESH_BINARY)
        label_image = cv.CreateImage(cv.GetSize(raw_image), cvblob.IPL_DEPTH_LABEL, 1)

        blobs = cvblob.Blobs()
        result = cvblob.Label(thresh_image, label_image, blobs)

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
            cvblob.RenderBlobs(label_image, blobs, raw_image, self.blobs_image, self.blob_mask, 1.0)
            blobs_rosimage = self.bridge.cv_to_imgmsg(self.blobs_image,encoding="passthrough")
            return blobs_list, blobs_rosimage
