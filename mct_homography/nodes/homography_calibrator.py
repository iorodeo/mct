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
from cv_bridge.cv_bridge import CvBridge 

# Services
from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse
from mct_msg_and_srv.srv import GetMatrix
from mct_msg_and_srv.srv import GetMatrixResponse

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
        WAITING  : (255,100,0),
        WORKING  : (0,0,255),
        FINISHED : (0,255,0) ,
        }

class HomographyCalibratorNode(object):

    def __init__(self,topic):

        self.state = WAITING  # There are 3 allowed states WAITING, WORKING, FINISHED
        self.topic = topic
        self.bridge = CvBridge()
        self.lock = threading.Lock()

        # Initialize data lists
        self.blobs_list = [] 
        self.image_points = [] 
        self.world_points = []

        # Set active target information and turn off leds
        target_info = mct_active_target.active_target_info()
        self.led_n_max = target_info[0] 
        self.led_m_max = target_info[1]
        self.led_max_power = target_info[2]
        self.led_n = 0
        self.led_m = 0
        self.led_power = 10 
        self.led_space_x = 1.5*0.0254
        self.led_space_y = 1.5*0.0254
        self.number_of_leds = self.led_n_max*self.led_m_max
        mct_active_target.off()

        # Wait count for image acquisition
        self.image_wait_number = 4
        self.image_wait_cnt = 0

        # Sleep periods for idle and wait count loops
        self.idle_sleep_dt = 0.1
        self.wait_sleep_dt = 0.005

        # Initialize blob finder
        self.blobFinder = BlobFinder()
        self.blobFinder.threshold = 200 
        self.blobFinder.filter_by_area = False
        self.blobFinder.min_area = 0
        self.blobFinder.max_area = 200

        # Initialize homography matrix and number of points required to solve for it
        self.homography_matrix = None
        self.num_points_required = 10

        # Set font and initial image information
        self.cv_text_font = cv.InitFont(cv.CV_FONT_HERSHEY_TRIPLEX, 0.8, 0.8,thickness=1)
        self.image_info= 'no data'

        rospy.init_node('homography_calibrator')

        # Subscription to image topic
        self.image_sub = rospy.Subscriber(self.topic,Image,self.image_callback)

        # Publications - blobs image and calibration progress image
        self.image_blobs_pub = rospy.Publisher('image_blobs', Image)
        self.image_calib_pub = rospy.Publisher('image_calibration', Image)

        # Services
        node_name = rospy.get_name()
        self.start_srv = rospy.Service(
                '{0}/start'.format(node_name),
                Empty,
                self.handle_start_srv
                )

        self.get_matrix_srv = rospy.Service(
                '{0}/get_matrix'.format(node_name),
                GetMatrix,
                self.handle_get_matrix_srv
                )

    def handle_start_srv(self,req):
        """
        Handles to start/restart the homography calibration procedure.
        """
        with self.lock:
            self.homography_matrix = None
            self.image_info = ''
            self.image_points = []
            self.world_points = []
            self.blobs_list = []
            self.state = WORKING
            self.led_n = 0
            self.led_m = 0
        return EmptyResponse()

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


    def image_callback(self,data):
        """
        Image topic callback function. Uses the blobFinder to find the blobs
        (leds) in the image. Publishes images showing the blobs and the
        calibration progress.
        """
        # Find blobs
        with self.lock:
            self.blobs_list, blobs_rosimage = self.blobFinder.findBlobs(data)
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
            message.append('{0}/{1} pts'.format(len(self.image_points),self.number_of_leds))
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
            # Need to check that we got enough done otherwise return to idle.
            if len(self.image_points) >= self.num_points_required:
                self.state = FINISHED 
                self.image_info = ''
            else:
                self.state = WAITING
                self.image_info = 'not enough data'

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
                mct_active_target.off()
                rospy.sleep(self.idle_sleep_dt)

class HomographyCalibrator(object):

    def __init__(self):
        self.have_cal = False

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    topic = sys.argv[1]
    node = HomographyCalibratorNode(topic)
    node.run()
