#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('mct_transform_2d')
import rospy
import numpy
import cv
import cv2
import sys
import threading
import mct_active_target
import functools
import scipy.optimize
from mct_blob_finder import BlobFinder
from mct_utilities import file_tools
from cv_bridge.cv_bridge import CvBridge 

# Services
from mct_msg_and_srv.srv import GetBool
from mct_msg_and_srv.srv import GetBoolResponse
from mct_msg_and_srv.srv import GetFlagAndMessage
from mct_msg_and_srv.srv import GetFlagAndMessageResponse
from mct_msg_and_srv.srv import GetTransform2d
from mct_msg_and_srv.srv import GetTransform2dResponse

# Messages
from sensor_msgs.msg import Image

WAITING = 0
WORKING = 1
FINISHED= 2

STATE_MESSAGE = {
        WAITING  : "Waiting",
        WORKING  : "Calibrating",
        FINISHED : "Finished",
        }

STATE_COLOR = {
        WAITING  : (0,255,255),
        WORKING  : (0,0,255),
        FINISHED : (0,255,0) ,
        }

class Transform_2D_Calibrator(object):

    def __init__(self, topic_0, topic_1):

        self.state = WAITING  # There are 3 allowed states WAITING, WORKING, FINISHED
        #self.state = WORKING  
        self.topics = topic_0, topic_1
        self.bridge = CvBridge()
        self.lock = threading.Lock()

        rospy.init_node('transform_2d_calibrator')
        self.node_name = rospy.get_name()

        # Initialize data lists
        self.blobs = {
                self.topics[0]: [], 
                self.topics[1]: [],
                }
        self.index_to_image = {
                self.topics[0]: {}, 
                self.topics[1]: {},
                }
        self.overlap_indices = [] 

        # Set active target information and turn off leds
        target_info = mct_active_target.active_target_info()
        self.led_n_max = target_info[0] 
        self.led_m_max = target_info[1]
        self.number_of_leds = self.led_n_max*self.led_m_max
        self.led_max_power = target_info[2]
        self.led_space_x = target_info[3] 
        self.led_space_y = target_info[3] 
        mct_active_target.off()

        # Current led indices
        self.led_n = 0
        self.led_m = 0

        # Wait counter for image acquisition

        self.image_wait_cnt = { 
                self.topics[0]: 0,
                self.topics[1]: 0,
                }

        # Sleep periods for idle and wait count loops
        self.idle_sleep_dt = 0.25
        self.wait_sleep_dt = 0.005

        # Led power setting
        self.led_power = rospy.get_param(
                '{0}/target/led_power'.format(self.node_name),
                10
                )  

        # Wait count for image acquisition
        self.image_wait_number = rospy.get_param(
                '{0}/image_wait_number'.format(self.node_name),
                4
                ) 

        # Initialize blob finder
        self.blobFinder = BlobFinder()
        self.blobFinder.threshold = rospy.get_param(
                '{0}/blob_finder/threshold'.format(self.node_name),
               150 
                ) 
        self.blobFinder.filter_by_area = rospy.get_param(
                '{0}/blob_finder/filter_by_area'.format(self.node_name), 
                False
                ) 
        self.blobFinder.min_area = rospy.get_param(
                '{0}/blob_finder/min_area'.format(self.node_name), 
                0
                )
        self.blobFinder.max_area = rospy.get_param(
                '{0}/blob_finder/max_area'.format(self.node_name),
                200
                ) 

        # Number of points required
        self.num_points_required = rospy.get_param(
                '{0}/num_points_required'.format(self.node_name), 
                10
                ) 
        self.transform = None
        self.transform_error = None

        # Get homography matrices
        self.homography_dict = {}
        for topic in self.topics:
            try:
                # Get the data from the parameter server
                rows = rospy.get_param('{0}/homography_matrix/rows')
                cols = rospy.get_param('{0}/homography_matrix/cols')
                data = rospy.get_param('{0}/homography_matrix/data')
            except KeyError:
                # Data is not on the parameter server - try getting it by reading the file
                camera = get_camera_from_topic(topic)
                homography = file_tools.read_homography_calibration(camera)
                rows = homography['rows']
                cols = homography['cols']
                data = homography['data']
           
            # Rearrange into numpy matrix
            homography_matrix = numpy.array(homography['data'])
            homography_matrix = homography_matrix.reshape((rows,cols))
            self.homography_dict[topic] = homography_matrix


        # Set font and initial image information
        self.cv_text_font = cv.InitFont(cv.CV_FONT_HERSHEY_TRIPLEX, 0.8, 0.8,thickness=1)
        self.image_info= 'no data'

        # Subscription to image topic
        image_callback_0 = functools.partial(self.image_callback, self.topics[0])
        image_callback_1 = functools.partial(self.image_callback, self.topics[1])
        self.image_sub = {
                self.topics[0] : rospy.Subscriber(self.topics[0], Image, image_callback_0),
                self.topics[1] : rospy.Subscriber(self.topics[1], Image, image_callback_1),
                }

        # Publications - bcalibration progress images for topics 0 and 1
        self.image_pub = {
                self.topics[0] : rospy.Publisher('image_transform_0', Image),
                self.topics[1] : rospy.Publisher('image_transform_1', Image),
                }

        # Services
        self.start_srv = rospy.Service(
                '{0}/start'.format(self.node_name),
                GetFlagAndMessage,
                self.handle_start_srv
                )

        self.is_calibrated_srv = rospy.Service(
                '{0}/is_calibrated'.format(self.node_name),
                GetBool,
                self.handle_is_calibrated_srv
                )

        self.get_transform_2d_srv = rospy.Service(
                '{0}/get_transform_2d'.format(self.node_name),
                GetTransform2d,
                self.handle_get_transform_2d_srv
                )


    def handle_get_transform_2d_srv(self,req):
        """
        Handles requests to get the transform found by the 
        """
        if self.transform is None:
            rotation = 0.0
            translation_x = 0.0
            translation_y = 0.1
        else:
            rotation = self.transform['rotation']
            translation_x = self.transform['translation_x']
            translation_y = self.transform['translation_y']
        return GetTransform2dResponse(rotation, translation_x, translation_y)


    def handle_start_srv(self,req):
        """
        Handles to start/restart the homography calibration procedure.
        """
        flag = True
        message = ''
        with self.lock:
            flag, message = mct_active_target.lock(self.node_name)
            if flag:
                self.image_info = ''
                self.state = WORKING
                self.led_n = 0
                self.led_m = 0
                self.blobs = {topic_0: [], topic_1: []}
                self.index_to_image = {topic_0: {}, topic_1: {}}
                self.overlap_indices = []
                self.transform = None
                self.transform_error = None
        return GetFlagAndMessageResponse(flag,message)

    def handle_is_calibrated_srv(self,req):
        """
        Handles requests for whether or not the transform calibration has been completed.
        """
        if self.transform is not None:
            value = True
        else:
            value = False
        return GetBoolResponse(value)


    def image_callback(self,topic,data):
        """
        Image topic callback function. Uses the blobFinder to find the blobs
        (leds) in the image. Publishes images showing the calibration progress.
        """
        # Find blobs
        with self.lock:

            if self.state == WORKING:
                self.blobs[topic], blobs_rosimage = self.blobFinder.findBlobs(data)
            else: 
                self.blobs[topic] = []

            if self.image_wait_cnt[topic] < self.image_wait_number:
                self.image_wait_cnt[topic] += 1

        # Create calibration data  image
        cv_image = self.bridge.imgmsg_to_cv(data,desired_encoding="passthrough")
        ipl_image = cv.GetImage(cv_image)
        calib_image = cv.CreateImage(cv.GetSize(ipl_image), cv.IPL_DEPTH_8U, 3)
        cv.CvtColor(ipl_image,calib_image,cv.CV_GRAY2BGR)

        # Add calibration markers to image
        color = STATE_COLOR[self.state]
        if self.state == WORKING:
            for image_point in self.index_to_image[topic].values():
                x,y = image_point
                cv.Circle(calib_image, (int(x),int(y)),3, color)
        else:
            for index in self.overlap_indices:
                x,y = self.index_to_image[topic][index]
                cv.Circle(calib_image, (int(x),int(y)),3, color)

        ## Add text to image
        message = [STATE_MESSAGE[self.state]]
        if self.state == WORKING or self.state==FINISHED:
            if self.state == WORKING:
                num_points_found = len(self.index_to_image[topic])
            else:
                num_points_found = len(self.overlap_indices)
            message.append('{0}/{1} pts'.format(num_points_found,self.get_led_count()))
        if self.image_info:
            message.append('- {0}'.format(self.image_info))
        if (self.state == FINISHED) and (self.transform_error is not None):
                message.append('- error {0:1.2f} mm'.format(1e3*self.transform_error))
        message = ' '.join(message)
        cv.PutText(calib_image,message,(10,25),self.cv_text_font,color)

        # Publish calibration progress image
        calib_rosimage = self.bridge.cv_to_imgmsg(calib_image,encoding="passthrough")
        self.image_pub[topic].publish(calib_rosimage)

    def wait_for_images(self):
        """
        Wait for 'image_wait_number' of images to be acquired.
        """
        with self.lock:
            for topic in self.topics:
                self.image_wait_cnt[topic] = 0
        done = False
        while not done:
            with self.lock:
                image_wait_cnt = min([self.image_wait_cnt[t] for t in self.topics])
            if image_wait_cnt >= self.image_wait_number:
                done = True
            rospy.sleep(self.wait_sleep_dt)

    def increment_led(self):
        """
        Increments the led array indices. When the last led is reached check to see if
        sufficient points have been captured for the homography calibratoin. If so, then
        the state is changed to FINISHED.  If insufficient points have been gathered then
        the state is changed back to WAITING.
        """

        if self.led_m < self.led_m_max-1 or self.led_n < self.led_n_max-1:
            if self.led_n < self.led_n_max-1:
                self.led_n += 1
            else:
                self.led_n  = 0
                self.led_m += 1
        else:

            # Turn off led and unlock active target
            mct_active_target.off()
            mct_active_target.unlock(self.node_name)

            # Need to check that we got enough done otherwise return to idle.
            self.state = FINISHED
            self.overlap_indices = self.get_overlap_indices()
            if len(self.overlap_indices) >= self.num_points_required:
                self.state = FINISHED
                self.image_info = ''
                print('finished')
            else:
                self.state = WAITING
                self.image_info = 'not enough data'
                self.transform = None
                print('not enough data')

    def get_overlap_indices(self):
        """
        Returns the overlaping points
        """
        indices_0 = self.index_to_image[self.topics[0]]
        indices_1 = self.index_to_image[self.topics[1]]
        overlap_indices = set(indices_0)
        overlap_indices.intersection_update(indices_1)
        return list(overlap_indices)


    def get_led_count(self):
        return self.led_n + self.led_m*self.led_n_max + 1

    def save_blob_data(self):
        """
        Save blob data found in images
        """
        with self.lock: 
            for topic in self.topics: 
                if len(self.blobs[topic]) == 1:  
                    blob = self.blobs[topic][0]
                    image_x = blob['centroid_x']
                    image_y = blob['centroid_y']
                    self.index_to_image[topic][ (self.led_n, self.led_m) ] = (image_x, image_y)

    def find_transform(self):
        """
        Find 2d transform (rotation + translation) from the world points found
        in topic[0] and topic[1] and thier respective homographies.
        """
        # Get world points for both topics for all points in overlap 
        world_points_dict = {}

        for topic in self.topics:

            # Get homography from image coordinates to world coordinates
            homography_matrix = self.homography_dict[topic]
            homography_matrix_t = homography_matrix.transpose()

            # Get image points for indices in overlap
            image_points = []
            for index in self.overlap_indices:
                image_points.append(self.index_to_image[topic][index])
            image_points = numpy.array(image_points)

            # Convert to homogenious coordinates and compute world coordinates
            image_points_hg = numpy.ones((image_points.shape[0],3))
            image_points_hg[:,:2] = image_points
            world_points_hg = numpy.dot(image_points_hg, homography_matrix_t)
            world_points = numpy.array(world_points_hg[:,:2])
            denom = numpy.zeros((world_points.shape[0],2))
            denom[:,0] = world_points_hg[:,2]
            denom[:,1] = world_points_hg[:,2]
            world_points = world_points/denom
            world_points_dict[topic] = world_points

        # Estimate translate as mean difference in point positions
        points_0 = world_points_dict[self.topics[0]]
        points_1 = world_points_dict[self.topics[1]]
        rotation_est, translation_est = estimate_transform_2d(points_0, points_1)

        rotation, translation, error = fit_transform_2d(
                points_0, 
                points_1, 
                rotation_est, 
                translation_est
                )

        self.transform = {
                'rotation'      : rotation, 
                'translation_x' : translation[0],
                'translation_y' : translation[1],
                }

        self.transform_error = error

   
    def run(self):
        """
        Node main function. When the state is WORKING this function activates
        the leds on the active calibration target one at a time. When all of
        the leds have been activated and if sufficient number of points have
        been collected the homography matrix is calculated.
        """

        while not rospy.is_shutdown():

            if self.state == WORKING:
                # Turn on current led and wait for images
                mct_active_target.set_led(self.led_n, self.led_m, self.led_power)
                self.wait_for_images()
                self.save_blob_data()
                self.increment_led()

            elif self.state == FINISHED and self.transform is None:
                # Calculate 2d transforms
                self.find_transform()
            else:
                rospy.sleep(self.idle_sleep_dt)

# ----------------------------------------------------------------------------------

def fit_transform_2d(points_0, points_1, rotation_est, translation_est):
    """
    Fits the 2d transformation from points
    """
    def error_func(x):
        dx, dy, angle = x[0], x[1], x[2]
        rot_matrix = get_rot_matrix(angle)
        rot_matrix_t = rot_matrix.transpose()
        points_1_pred = numpy.dot(points_0,rot_matrix_t) + numpy.array([dx,dy])
        error = (points_1_pred - points_1)**2
        error = numpy.sqrt(error.sum(axis=1))
        return error.mean()

    # Find best least squares fit using downhill simplex
    x0 = numpy.array([translation_est[0], translation_est[1], rotation_est])
    result = scipy.optimize.fmin(error_func, x0,disp=False, full_output=True)
    sol, error = result[0], result[1]
    rotation, translation = sol[2], sol[:2]
    return rotation, translation, error


def estimate_transform_2d(points_0, points_1): 
    """
    Estimates 2d transformation from corresponding points_0 and points_1
    """

    # Estimate rotation - as mean angle between a and b vectors
    a = points_0 - points_0[0,:]
    b = points_1 - points_1[0,:]
    a = a[1:,:]
    b = b[1:,:]
    a_dot_b = a[:,0]*b[:,0] + a[:,1]*b[:,1]
    a_cross_b = a[:,0]*b[:,1] - a[:,1]*b[:,0]
    rotation_est = numpy.arctan2(a_cross_b, a_dot_b)
    rotation_est = rotation_est.mean()

    # Un-rotate and estimate translation
    rot_matrix = get_rot_matrix(rotation_est)
    rot_matrix_t = rot_matrix.transpose()
    points_0_rot = numpy.dot(points_0,rot_matrix_t)
    translation_est = points_1 - points_0_rot
    translation_est = translation_est.mean(axis=0)
    return rotation_est, translation_est

def get_rot_matrix(angle): 
    """
    Returns 2d rotation matrix given the rotation angle
    """
    rot_matrix = numpy.array([ 
        [numpy.cos(angle), -numpy.sin(angle)], 
        [numpy.sin(angle),  numpy.cos(angle)], 
        ])
    return rot_matrix

def get_camera_from_topic(topic):
    """
    Gets the camera name from the image topic. Note, assumes that topic is
    of the form /<machine>/<camera>/...

    """
    topic_split = topic.split('/')
    camera = topic_split[2]
    return camera


# -----------------------------------------------------------------------------
if __name__ == '__main__':
    topic_0 = sys.argv[1]
    topic_1 = sys.argv[2]
    node = Transform_2D_Calibrator(topic_0, topic_1)
    node.run()
