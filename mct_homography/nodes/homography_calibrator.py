#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('mct_homography')
import rospy
import numpy
import cv
import cv2
import sys
import threading
import mct_active_target
from mct_blob_finder import BlobFinder
from mct_utilities import file_tools
from cv_bridge.cv_bridge import CvBridge 

# Services
from mct_msg_and_srv.srv import GetMatrix
from mct_msg_and_srv.srv import GetMatrixResponse
from mct_msg_and_srv.srv import GetBool
from mct_msg_and_srv.srv import GetBoolResponse
from mct_msg_and_srv.srv import GetFlagAndMessage
from mct_msg_and_srv.srv import GetFlagAndMessageResponse

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

class HomographyCalibratorNode(object):

    def __init__(self,topic):

        self.state = WAITING  # There are 3 allowed states WAITING, WORKING, FINISHED
        self.topic = topic
        self.bridge = CvBridge()
        self.lock = threading.Lock()

        rospy.init_node('homography_calibrator')
        self.node_name = rospy.get_name()

        # Initialize data lists
        self.blobs_list = [] 
        self.image_points = [] 
        self.world_points = []

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

        # Led power setting
        params_namespace = '/homography_calibrator_params'
        self.led_power = rospy.get_param(
                '{0}/target/led_power'.format(params_namespace),
                10
                )  

        # Wait count for image acquisition
        self.image_wait_number = rospy.get_param(
                '{0}/image_wait_number'.format(params_namespace),
                4
                ) 
        self.image_wait_cnt = 0

        # Sleep periods for idle and wait count loops
        self.idle_sleep_dt = 0.25
        self.wait_sleep_dt = 0.005

        # Initialize blob finder
        self.blobFinder = BlobFinder()
        self.blobFinder.threshold = rospy.get_param(
                '{0}/blob_finder/threshold'.format(params_namespace),
                200
                ) 
        self.blobFinder.filter_by_area = rospy.get_param(
                '{0}/blob_finder/filter_by_area'.format(params_namespace), 
                False
                ) 
        self.blobFinder.min_area = rospy.get_param(
                '{0}/blob_finder/min_area'.format(params_namespace), 
                0
                )
        self.blobFinder.max_area = rospy.get_param(
                '{0}/blob_finder/max_area'.format(params_namespace),
                200
                ) 

        # Initialize homography matrix and number of points required to solve for it
        self.homography_matrix = None
        self.num_points_required = rospy.get_param(
                '{0}/num_points_required'.format(params_namespace), 
                10
                ) 

        # Set font and initial image information
        self.cv_text_font = cv.InitFont(cv.CV_FONT_HERSHEY_TRIPLEX, 0.8, 0.8,thickness=1)
        self.image_info= 'no data'


        # Subscription to image topic
        self.image_sub = rospy.Subscriber(self.topic,Image,self.image_callback)

        # Publications - blobs image and calibration progress image
        self.image_blobs_pub = rospy.Publisher('image_homography_blobs', Image)
        self.image_calib_pub = rospy.Publisher('image_homography_calibration', Image)

        # Services
        self.start_srv = rospy.Service(
                '{0}/start'.format(self.node_name),
                GetFlagAndMessage,
                self.handle_start_srv
                )

        self.get_matrix_srv = rospy.Service(
                '{0}/get_matrix'.format(self.node_name),
                GetMatrix,
                self.handle_get_matrix_srv
                )

        self.is_calibrated_srv = rospy.Service(
                '{0}/is_calibrated'.format(self.node_name),
                GetBool,
                self.handle_is_calibrated_srv
                )

    def handle_start_srv(self,req):
        """
        Handles to start/restart the homography calibration procedure.
        """
        flag = True
        message = ''
        with self.lock:
            flag, message = mct_active_target.lock(self.node_name)
            if flag:
                self.homography_matrix = None
                self.image_info = ''
                self.image_points = []
                self.world_points = []
                self.blobs_list = []
                self.state = WORKING
                self.led_n = 0
                self.led_m = 0
        return GetFlagAndMessageResponse(flag,message)

    def handle_get_matrix_srv(self,req):
        """
        Returns the homography matrix. If the homography matrix has not yet be computed the 
        identity matirx is returned.
        """
        if self.homography_matrix is None:
            data = [1,0,0, 0,1,0, 0,0,1]
        else:
            data = self.homography_matrix.reshape((9,))
        return GetMatrixResponse(3,3,data)

    def handle_is_calibrated_srv(self,req):
        """
        Handles requests for whether or not the homography calibration has been completed.
        """
        if self.homography_matrix is not None:
            value = True
        else:
            value = False
        return GetBoolResponse(value)


    def image_callback(self,data):
        """
        Image topic callback function. Uses the blobFinder to find the blobs
        (leds) in the image. Publishes images showing the blobs and the
        calibration progress.
        """
        # Find blobs
        with self.lock:

            if self.state == WORKING:
                self.blobs_list, blobs_rosimage = self.blobFinder.findBlobs(data)
            else: 
                self.blobs_list = []
                blobs_rosimage = data

            if self.image_wait_cnt < self.image_wait_number:
                self.image_wait_cnt += 1

        self.image_blobs_pub.publish(blobs_rosimage)

        # Create calibration data  image
        cv_image = self.bridge.imgmsg_to_cv(data,desired_encoding="passthrough")
        ipl_image = cv.GetImage(cv_image)
        calib_image = cv.CreateImage(cv.GetSize(ipl_image), cv.IPL_DEPTH_8U, 3)
        cv.CvtColor(ipl_image,calib_image,cv.CV_GRAY2BGR)

        # Add calibration markers to image
        color = STATE_COLOR[self.state]
        for x,y in self.image_points:
            cv.Circle(calib_image, (int(x),int(y)),3, color)

        ## Add text to image
        message = [STATE_MESSAGE[self.state]]

        if self.state == WORKING or self.state==FINISHED:
            #message.append('{0}/{1} pts'.format(len(self.image_points),self.number_of_leds))
            message.append('{0}/{1} pts'.format(len(self.image_points),self.get_led_count()))
        if self.image_info:
            message.append('- {0}'.format(self.image_info))
        message = ' '.join(message)
        cv.PutText(calib_image,message,(10,25),self.cv_text_font,color)

        # Publish calibration progress image
        calib_rosimage = self.bridge.cv_to_imgmsg(calib_image,encoding="passthrough")
        self.image_calib_pub.publish(calib_rosimage)

    def wait_for_images(self):
        """
        Wait for 'image_wait_number' of images to be acquired.
        """
        with self.lock:
            self.image_wait_cnt = 0
        done = False
        while not done:
            with self.lock:
                image_wait_cnt = self.image_wait_cnt
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
            if len(self.image_points) >= self.num_points_required:
                self.state = FINISHED 
                self.image_info = ''
            else:
                self.state = WAITING
                self.image_info = 'not enough data'
                self.homography_matrix = None

    def get_led_count(self):
        return self.led_n + self.led_m*self.led_n_max + 1

    def run(self):
        """
        Node main function. When the state is WORKING this function activates the leds on the
        active calibration target one at a time. When all of the leds have been activated and
        if sufficient number of points have been collected the homography matrix is calculated.
        """

        while not rospy.is_shutdown():

            if self.state == WORKING:

                # Turn on current led and wait for images
                mct_active_target.set_led(self.led_n, self.led_m, self.led_power)
                self.wait_for_images()

                if len(self.blobs_list) == 1:

                    # Get centroid of blob and add to image point list
                    blob = self.blobs_list[0]
                    image_x = blob['centroid_x']
                    image_y = blob['centroid_y']
                    self.image_points.append((image_x, image_y))

                    # Compute coordinates of the led in the world plane
                    world_x = self.led_n*self.led_space_x
                    world_y = self.led_m*self.led_space_y
                    self.world_points.append((world_x, world_y))

                self.increment_led()

            elif self.state == FINISHED and self.homography_matrix is None:

                    # Find the homography transformation
                    image_points = numpy.array(self.image_points)
                    world_points = numpy.array(self.world_points)
                    result = cv2.findHomography(image_points, world_points, cv.CV_RANSAC)
                    self.homography_matrix, mask = result 

                    # Compute the mean reprojection error
                    image_points_hg = numpy.ones((image_points.shape[0],3))
                    image_points_hg[:,:2] = image_points
                    world_points_hg = numpy.ones((world_points.shape[0],3))
                    world_points_hg[:,:2] = world_points
                    homography_matrix_t = self.homography_matrix.transpose()
                    world_points_hg_pred = numpy.dot(image_points_hg, homography_matrix_t)
                    denom = numpy.zeros((world_points.shape[0],2))
                    denom[:,0] = world_points_hg_pred[:,2]
                    denom[:,1] = world_points_hg_pred[:,2]
                    world_points_pred = world_points_hg_pred[:,:2]/denom
                    error = (world_points - world_points_pred)**2
                    error = error.sum(axis=1)
                    error = numpy.sqrt(error)
                    error = error.mean()
                    self.image_info = 'error {0:1.2f} mm'.format(1e3*error)
            else:
                rospy.sleep(self.idle_sleep_dt)


# -----------------------------------------------------------------------------
if __name__ == '__main__':
    topic = sys.argv[1]
    node = HomographyCalibratorNode(topic)
    node.run()
