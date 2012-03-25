#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('mct_blob_finder')
import rospy
import sys
import threading
import math

import numpy
import scipy.optimize

import cv
from cv_bridge.cv_bridge import CvBridge 
from mct_blob_finder import BlobFinder
from mct_utilities import file_tools

# Messages
from sensor_msgs.msg import Image
from mct_msg_and_srv.msg import BlobData
from mct_msg_and_srv.msg import Blob
from std_msgs.msg import Float32

# Services
from mct_msg_and_srv.srv import BlobFinderSetParam
from mct_msg_and_srv.srv import BlobFinderSetParamResponse
from mct_msg_and_srv.srv import BlobFinderGetParam
from mct_msg_and_srv.srv import BlobFinderGetParamResponse

class ThreePointTracker(object):

    def __init__(self,topic=None):
        self.topic = topic
        self.lock = threading.Lock()
        self.bridge = CvBridge()
        self.camera = get_camera_from_topic(self.topic)
        camera_calibration = file_tools.read_camera_calibration(self.camera)
        self.camera_matrix = get_camera_matrix(camera_calibration)

        # Blob finder parameters
        self.blobFinder = BlobFinder()
        self.blobFinder.threshold = 150
        self.blobFinder.filter_by_area = False
        self.blobFinder.min_area = 0
        self.blobFinder.max_area = 200

        # Tracking parameters
        self.tracking_pts_dist = (0.0, 0.04774, 0.07019)
        self.tracking_pts_colors = [(0,0,255), (0,255,0), (0,255,255)]

        rospy.init_node('blob_finder')
        self.ready = False
        self.image_sub = rospy.Subscriber(self.topic,Image,self.image_callback)

        self.image_tracking_pts_pub = rospy.Publisher('image_tracking_pts', Image)
        self.image_blobs_pub = rospy.Publisher('image_blobs', Image)
        self.blob_data_pub = rospy.Publisher('blob_data', BlobData)

        self.devel_pub = rospy.Publisher('devel_data', Float32)

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

        #blobs_list = get_synthetic_blobs(self.tracking_pts_dist, self.camera_matrix)
        num_blobs = len(blobs_list)
        if num_blobs == 3:

            #print('3 pts found')
            uv_list = self.get_sorted_uv_points(blobs_list)
            t,w = get_object_pose(1.0,2.2, uv_list, self.tracking_pts_dist, self.camera_matrix)
            uv_list_pred = get_uv_list_from_vects(t,w, self.tracking_pts_dist, self.camera_matrix)
            elev = math.asin(w[2])
            head = math.atan2(w[1], w[0])
            elev_deg = 180.0*elev/math.pi
            head_deg = 180.0*head/math.pi

            #print('w', ['{0:1.3f}'.format(x) for x in w])
            #print('t', ['{0:1.3f}'.format(x) for x in t])
            print('head: {0:1.3f}'.format(head_deg))
            print('elev: {0:1.3f}'.format(elev_deg))
            print('tx:   {0:1.3f}'.format(t[0]))
            print('ty:   {0:1.3f}'.format(t[1]))
            print('tz:   {0:1.3f}'.format(t[2]))
            print()
            self.devel_pub.publish(t[2])

            # Draw points on tracking points image
            for i, uv in enumerate(uv_list):
                color = self.tracking_pts_colors[i]
                u,v = uv 
                #cv.Circle(tracking_pts_image, (int(u),int(v)),3, color)
                cv.Circle(tracking_pts_image, (int(u),int(v)),3, (0,255,0))

            for i, uv in enumerate(uv_list_pred):
                color = self.tracking_pts_colors[i]
                u,v = uv 
                #cv.Circle(tracking_pts_image, (int(u),int(v)),3, color)
                cv.Circle(tracking_pts_image, (int(u),int(v)),3, (0,0,255))
        else:
            print('3 pts not found ', end='')
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

def get_lstsq_pose_vects(tz, uv_list, pts_dist, camera_matrix):
    """
    Calculates the best fit pose vectors (t,w) for the object given the depth tz.
    p_i = t + d_i * w
    * tx,ty are computed using the camera model 
    * vx,vy,vz are calculated by solvng via lstsq the linear set of equations Aw = -Bt 
    where A and B come from putting the points  p_i , i=0,1,2 into the camera model.  
    """
    # Extract the image points
    u = [uu for uu,vv in uv_list]
    v = [vv for uu,vv in uv_list]

    # Get the camera parametes
    fx = camera_matrix[0,0]
    fy = camera_matrix[1,1]
    cx = camera_matrix[0,2]
    cy = camera_matrix[1,2]

    # Get the distance from  point 0 to points 1 and 2
    d0 = pts_dist[1] 
    d1 = pts_dist[2]

    # Computer x and y translations
    tx = (u[0] - cx)*(tz/fx)
    ty = (v[0] - cy)*(tz/fy)
    t = numpy.array([tx,ty,tz])

    A = numpy.array([
        [-fx*d0,     0.0,   d0*(u[1] - cx)],    
        [   0.0,  -fy*d0,   d0*(v[1] - cy)],    
        [-fx*d1,     0.0,   d1*(u[2] - cx)],    
        [   0.0,  -fy*d1,   d1*(v[2] - cy)],    
        ])

    B = numpy.array([
        [-fx,    0.0,    u[1] - cx],
        [0.0,    -fy,    v[1] - cy],
        [-fx,    0.0,    u[2] - cx],
        [0.0,    -fy,    v[2] - cy],
        ])

    b = -numpy.dot(B,t)
    result = numpy.linalg.lstsq(A,b)
    w = list(result[0])
    #resid = float(result[1])
    #print(resid)
    return t, w

def get_object_pose(z_min, z_max, uv_list, pts_dist, camera_matrix):
    """
    Finds the pose of the obj (assuming three colinear points). 

    z_min    = minimum z-coord. for pose optimization. 
    z_max    = maximum z-coord. for pose optimization.
    uv_list  = list of 3 camera image points (u_i, v_i)  
    pts_dist = distance of the object points along the x-axis, unrotated and untranslated
    camera_matrix = the camera matrix.

    returns 

    t = optimal translation vector (3,)
    w = optimal orientation vector (3,)

    such that 

    p_i = t + d_i*w where d_i=pts_dist[i]

    # -----------------------------------------
    Note, currently pts_dist[0] must be zero. 
    # -----------------------------------------
    """

    def cost_func(tz):
        """
        Cost function for finding pose using get_lstsq_pose_vect. Returns 
        the squared error between  w and the unit 2-sphere.
        """
        t,w = get_lstsq_pose_vects(tz, uv_list, pts_dist, camera_matrix)
        w_mag = numpy.sqrt(numpy.dot(w,w))
        w = w/w_mag

        ## Extract the image points
        #u = [uu for uu,vv in uv_list]
        #v = [vv for uu,vv in uv_list]

        ## Get the camera parametes
        #fx = camera_matrix[0,0]
        #fy = camera_matrix[1,1]
        #cx = camera_matrix[0,2]
        #cy = camera_matrix[1,2]

        ## Get the distance from  point 0 to points 1 and 2
        #d0 = pts_dist[1] 
        #d1 = pts_dist[2]

        #A = numpy.array([
        #    [-fx*d0,     0.0,   d0*(u[1] - cx)],    
        #    [   0.0,  -fy*d0,   d0*(v[1] - cy)],    
        #    [-fx*d1,     0.0,   d1*(u[2] - cx)],    
        #    [   0.0,  -fy*d1,   d1*(v[2] - cy)],    
        #    ])
        #B = numpy.array([
        #    [-fx,    0.0,    u[1] - cx],
        #    [0.0,    -fy,    v[1] - cy],
        #    [-fx,    0.0,    u[2] - cx],
        #    [0.0,    -fy,    v[2] - cy],
        #    ])
        #a = numpy.dot(A,w)
        #b = numpy.dot(B,t)
        #c = a + b
        #cost = c**2
        #cost = cost.sum()

        cost = (1.0 - w_mag)**2
        return cost

    # Find optimal z coord.
    tz_opt =  scipy.optimize.fminbound(cost_func, z_min, z_max)
    t,w = get_lstsq_pose_vects(tz_opt, uv_list, pts_dist, camera_matrix)

    # Normalize w to ensure unit length - pre-caution 
    w_mag = numpy.sqrt(numpy.dot(w,w))
    w = w/w_mag
    return t,w

def get_uv_list_from_vects(t,w, pts_dist, camera_matrix):
    w_mag = numpy.sqrt(numpy.dot(w,w)) 
    w = w/w_mag
    #print('in w', ['{0:1.3f}'.format(x) for x in w])
    #print('in t', ['{0:1.3f}'.format(x) for x in t])
    fx = camera_matrix[0,0]
    fy = camera_matrix[1,1]
    cx = camera_matrix[0,2]
    cy = camera_matrix[1,2]
    d0 = pts_dist[1]
    d1 = pts_dist[2]
    p0 = t 
    p1 = t + d0*w
    p2 = t + d1*w
    p_list = (p0,p1,p2)
    # Create camera image points
    uv_list = []
    for p in p_list:
        x,y,z = p
        u = fx*x/z + cx
        v = fy*y/z + cy
        uv_list.append((u,v))
    return  uv_list

def get_synthetic_blobs(t,w,pts_dist,camera_matrix):
    uv_list = get_synthetic_uv_list(pts_dist, camera_matrix)
    blobs_list = []
    for u,v in uv_list:
        blob = {'centroid_x': u, 'centroid_y': v}
        blobs_list.append(blob)
    return blobs_list


#def fit_uv_list(uv_list, pts_dist):
#    u = [uu for uu,vv in uv_list]
#    v = [vv for uu,vv in uv_list]
#    u_span = max(u) - min(u)
#    v_span = max(v) - min(v)
#    if u_span >= v_span:
#        x,y = u,v
#    else:
#        x,y = v,u
#    fit = scipy.polyfit(x,y,1)
#    y_fit = scipy.polyval(fit,x)
#    if u_span >= v_span:
#        u_fit, v_fit  = x, y_fit
#    else:
#        u_fit, v_fit = y_fit, x
#    uv_list_fit = zip(u_fit,v_fit)
#    return uv_list_fit
    
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
    import pylab
    import time

    if 1:
        topic = sys.argv[1]
        node = ThreePointTracker(topic)
        node.run()

    if 0:

        t = numpy.array([0,0,1])
        w = numpy.array([1,1,0])

        camera_calibration = file_tools.read_camera_calibration('camera_1')
        camera_matrix = get_camera_matrix(camera_calibration)
        pts_dist = (0.0, 0.04774, 0.07019)
        uv_list = get_uv_list_from_vects(t,w,pts_dist,camea_matrix)
        t0 = time.time()
        get_object_pose(1.0,3.0,uv_list, pts_dist, camera_matrix)
        t1 = time.time()
        dt = t1-t0
        print(1/dt)



        #tz_array = pylab.linspace(1.5,2.5,50) 
        #cost_list = []
        #for tz in tz_array:
        #    cost = get_lstsq_pose_vects(tz, uv_list, pts_dist, camera_matrix)
        #    cost_list.append(cost)

        #pylab.plot(tz_array, cost_list)
        #pylab.show()


