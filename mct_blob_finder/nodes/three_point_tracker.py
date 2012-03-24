#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('mct_blob_finder')
import rospy
import sys
import threading
import math

import numpy

import cv
from cv_bridge.cv_bridge import CvBridge 
from mct_blob_finder import BlobFinder
from mct_utilities import file_tools

# Messages
from sensor_msgs.msg import Image
from mct_msg_and_srv.msg import BlobData
from mct_msg_and_srv.msg import Blob

# Services
from mct_msg_and_srv.srv import BlobFinderSetParam
from mct_msg_and_srv.srv import BlobFinderSetParamResponse
from mct_msg_and_srv.srv import BlobFinderGetParam
from mct_msg_and_srv.srv import BlobFinderGetParamResponse

class BlobFinderNode(object):

    def __init__(self,topic=None):
        self.topic = topic
        self.lock = threading.Lock()
        self.bridge = CvBridge()
        self.camera = get_camera_from_topic(self.topic)
        camera_calibration = file_tools.read_camera_calibration(self.camera)
        for k,v in camera_calibration.iteritems():
            print(k,v)
        self.camera_matrix = get_camera_matrix(camera_calibration)

        # Blob finder parameters
        self.blobFinder = BlobFinder()
        self.blobFinder.threshold = 150
        self.blobFinder.filter_by_area = False
        self.blobFinder.min_area = 0
        self.blobFinder.max_area = 200

        # Tracking parameters
        self.tracking_pts_pos = (0.0, 0.04774, 0.07019)
        self.tracking_pts_colors = [(0,0,255), (0,255,0), (0,255,255)]

        rospy.init_node('blob_finder')
        self.ready = False
        self.image_sub = rospy.Subscriber(self.topic,Image,self.image_callback)

        self.image_tracking_pts_pub = rospy.Publisher('image_tracking_pts', Image)
        self.image_blobs_pub = rospy.Publisher('image_blobs', Image)
        self.blob_data_pub = rospy.Publisher('blob_data', BlobData)

        node_name = rospy.get_name()
        self.set_param_srv = rospy.Service( 
                '{0}/set_param'.format(node_name), 
                BlobFinderSetParam, 
                self.handle_set_param_srv
                )
        self.get_param_srv = rospy.Service( 
                '{0}/get_param'.format(node_name), 
                BlobFinderGetParam, 
                self.handle_get_param_srv
                )
        self.ready = True
        
    def handle_set_param_srv(self, req):
        """
        Handles requests to set the blob finder's parameters. Currently this
        is just the threshold used for binarizing the image.
        """
        with self.lock:
            self.blobFinder.threshold = req.threshold
            self.blobFinder.filter_by_area = req.filter_by_area
            self.blobFinder.min_area = req.min_area
            self.blobFinder.max_area = req.max_area
        return BlobFinderSetParamResponse(True,'')

    def handle_get_param_srv(self,req):
        """
        Handles requests for the blob finders parameters
        """
        with self.lock:
            threshold = self.blobFinder.threshold
            filter_by_area = self.blobFinder.filter_by_area
            min_area = self.blobFinder.min_area
            max_area = self.blobFinder.max_area
        resp_args = (threshold, filter_by_area, min_area, max_area)
        return  BlobFinderGetParamResponse(*resp_args)


    def image_callback(self,data):
        """
        Callback for image topic subscription - finds blobs in image.
        """
        if not self.ready:
            return 

        with self.lock:
            blobs_list, blobs_rosimage = self.blobFinder.findBlobs(data)

        # ---------------------------------------------------------------------

        # Create trakcing points  image
        cv_image = self.bridge.imgmsg_to_cv(data,desired_encoding="passthrough")
        ipl_image = cv.GetImage(cv_image)
        tracking_pts_image = cv.CreateImage(cv.GetSize(ipl_image), cv.IPL_DEPTH_8U, 3)
        cv.CvtColor(ipl_image,tracking_pts_image,cv.CV_GRAY2BGR)

        blobs_list = get_synthetic_blobs(self.camera_matrix, self.tracking_pts_pos)
        num_blobs = len(blobs_list)
        if num_blobs == 3:

            print('3 pts found')
            uv_list = self.get_sorted_uv_points(blobs_list)
            print('uv_list: ', uv_list)
            #uv_list_synth = self.get_sorted_uv_points(blobs_list_synth)
            #print('uv_list synth: ', uv_list_synth)
            
            get_3pt_object_pose(uv_list, self.camera_matrix, self.tracking_pts_pos)
            #get_3pt_object_pose(uv_list_synth, self.camera_matrix, self.tracking_pts_pos)

            # Draw points on tracking points image
            for i, uv in enumerate(uv_list):
                color = self.tracking_pts_colors[i]
                u,v = uv 
                cv.Circle(tracking_pts_image, (int(u),int(v)),3, color)
        else:
            print('3 pts not found', end='')
            if num_blobs > 3:
                print('too many blobs')
            else:
                print('too few blobs')


            
        # Publish calibration progress image
        ros_image = self.bridge.cv_to_imgmsg(tracking_pts_image,encoding="passthrough")
        self.image_tracking_pts_pub.publish(ros_image)

        # ---------------------------------------------------------------------

        ## Publish image of blobs
        #self.image_blobs_pub.publish(blobs_rosimage)

        ## Create the blob data message and publish
        #blob_data_msg = BlobData()
        #blob_data_msg.header = data.header
        #blob_data_msg.number_of_blobs = len(blobs_list)
        #for b in blobs_list:
        #    blob_msg = Blob()
        #    for k, v in b.iteritems():
        #        setattr(blob_msg,k,v)
        #    blob_data_msg.blob.append(blob_msg)
        #self.blob_data_pub.publish(blob_data_msg)


    def get_object_pose(self,uv_list):
        """
        Calculates pose of three point object
        """
        pass

    def get_sorted_uv_points(self,blobs_list):
        """
        For blob lists with three blobs finds the identities of the blobs based on
        colinearity and distance between the points.
        """
        assert len(blobs_list) == 3, 'blobs list must contain only thre blobs'
        # Get x and y point lists
        u_list = [blob['centroid_x'] for blob in blobs_list]
        v_list = [blob['centroid_y'] for blob in blobs_list]

        # Determine x and y span
        u_min, u_max = min(u_list), max(u_list)
        v_min, v_max = min(v_list), max(v_list)
        u_span = u_max - u_min
        v_span = v_max - v_min

        # Use max span to sort points
        if u_span >= v_span:
            uv_list = zip(u_list, v_list)
            uv_list.sort()
            u_list = [u for u,v in uv_list]
            v_list = [v for u,v in uv_list]
        else:
            vu_list = zip(v_list, u_list)
            vu_list.sort()
            u_list = [u for v,u in vu_list]
            v_list = [v for v,u in vu_list]

        # #####################################################################
        # Look at distances and sort so that the large gap occurs first
        # Note currently assuming the largest gap occurs first this should 
        # really come from the tracking_pts_pos 
        # #####################################################################
        uv_list= zip(u_list,v_list)
        dist_0_to_1 = distance_2d(uv_list[0], uv_list[1])
        dist_1_to_2 = distance_2d(uv_list[1], uv_list[2])
        if dist_0_to_1 <= dist_1_to_2:
            uv_list.reverse()
        return uv_list

    def run(self):
        rospy.spin()


# -----------------------------------------------------------------------------

def get_3pt_object_pose(image_pts, camera_matrix, pts_pos):
    """
    """
    u = [uu for uu,vv in image_pts]
    v = [vv for uu,vv in image_pts]
    fx = camera_matrix[0,0]
    fy = camera_matrix[1,1]
    cx = camera_matrix[0,2]
    cy = camera_matrix[1,2]
    d0 = pts_pos[1]
    d1 = pts_pos[2]
    span_u = max(u) - min(u)
    span_v = max(v) - min(v)
    if span_u >= span_v:
        sign = numpy.sign(u[-1]-u[0])
        print('1')
    else:
        print('2')
        sign = numpy.sign(v[-1]-v[0])

    problem_matrix = numpy.array([
        [   0.0,     0.0,              0.0,    -fx,    0.0,    u[0] - cx], 
        [   0.0,     0.0,              0.0,    0.0,    -fy,    v[0] - cy],
        [-fx*d0,     0.0,   d0*(u[1] + cx),    -fx,    0.0,    u[1] - cx],
        [   0.0,  -fy*d0,   d0*(v[1] + cy),    0.0,    -fy,    v[1] - cy],
        [-fx*d1,     0.0,   d1*(u[2] + cx),    -fx,    0.0,    u[2] - cx],
        [   0.0,  -fy*d1,   d1*(v[2] + cy),    0.0,    -fy,    v[2] - cy],
        ])

    # First pass to solve for orientation
    if span_u >= span_v:
        print('1')
        A = problem_matrix[:,1:]
        b = -sign*problem_matrix[:,0]
    else:
        print('2')
        A0 = problem_matrix[:,0:1]
        A1 = problem_matrix[:,2:]
        A = numpy.concatenate((A0,A1),axis=1)
        b = -sign*problem_matrix[:,1]
    result = numpy.linalg.lstsq(A,b)
    sol = result[0]

    if span_u >= span_v:
        print('1')
        v_orien = numpy.array([1.0*sign,sol[0],sol[1]]) 
        v_orien_mag = numpy.sqrt(numpy.dot(v_orien,v_orien)) 
        v_orien = v_orien/v_orien_mag
    else:
        print('2')
        v_orien = numpy.array([sol[0],1.0*sign,sol[1]])
        v_orien_mag = numpy.sqrt(numpy.dot(v_orien,v_orien)) 
        v_orien = v_orien/v_orien_mag

    # Second pass solve for translation
    A = problem_matrix[:,3:]
    B = problem_matrix[:,:3]
    b = -numpy.dot(B,v_orien)
    result = numpy.linalg.lstsq(A,b)
    v_trans = result[0]

    print('out v_orien', ['{0:1.2f}'.format(x) for x in v_orien])
    print('out v_trans', ['{0:1.2f}'.format(x) for x in v_trans])
    print()

def get_synthetic_blobs(camera_matrix,pts_pos):

    v_orien = numpy.array([1,-0.9,0])
    v_trans = numpy.array([0.090,0.04,1.95])
    v_orien_mag = numpy.sqrt(numpy.dot(v_orien,v_orien)) 
    v_orien = v_orien/v_orien_mag

    print('in v_orien', ['{0:1.2f}'.format(x) for x in v_orien])
    print('in v_trans', ['{0:1.2f}'.format(x) for x in v_trans])

    fx = camera_matrix[0,0]
    fy = camera_matrix[1,1]
    cx = camera_matrix[0,2]
    cy = camera_matrix[1,2]
    d0 = pts_pos[1]
    d1 = pts_pos[2]

    p0 = v_trans 
    p1 = v_trans + d0*v_orien
    p2 = v_trans + d1*v_orien
    p_list = (p0,p1,p2)

    # Create camera image points
    uv_list = []
    for p in p_list:
        u = fx*p[0]/p[2] + cx
        v = fx*p[1]/p[2] + cy
        uv_list.append((u,v))

    blobs_list = []
    for u,v in uv_list:
        blob = {'centroid_x': u, 'centroid_y': v}
        blobs_list.append(blob)
    return blobs_list

def distance_2d(p,q):
    return math.sqrt((p[0]-q[0])**2 + (p[1]-q[1])**2)

def get_camera_from_topic(topic):
    """
    Returns the camera name given the image topic.
    """
    topic_split = topic.split('/')
    return topic_split[2]

def get_matrix_from_calibration(name, calibration): 
    """
    Returns the camera calibration matrix given the ROS camera calibration
    dictionary
    """
    data = calibration[name]['data']
    rows = calibration[name]['rows']
    cols = calibration[name]['cols']
    matrix = numpy.array(data).reshape((rows,cols))
    return matrix

def get_camera_matrix(calibration):
    return get_matrix_from_calibration('camera_matrix',calibration)


# -----------------------------------------------------------------------------
if __name__ == '__main__':

    topic = sys.argv[1]
    node = BlobFinderNode(topic)
    node.run()
