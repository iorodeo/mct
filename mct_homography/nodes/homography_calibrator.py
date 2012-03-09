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

# Messages
from sensor_msgs.msg import Image


class HomographyCalibratorNode(object):

    def __init__(self,topic):

        self.topic = topic
        self.state = 'calibrating' # There are 3 allowed states 'idle', 'calibrating', 'done'
        self.lock = threading.Lock()
        self.bridge = CvBridge()

        # Initialize data lists
        self.blobs_list = [] 
        self.image_points = [] 
        self.world_points = []

        # Led active target information
        target_info = mct_active_target.active_target_info()
        self.led_n_max = target_info[0] 
        self.led_m_max = target_info[1]
        self.led_max_power = target_info[2]
        self.led_n = 0
        self.led_m = 0
        self.led_power = 10 
        self.led_space_x = 1.5*2.54
        self.led_space_y = 1.5*2.54

        # Wait count for image acquisition
        self.image_wait_number = 3
        self.image_wait_cnt = 0

        # Sleep periods for idle and wait count loops
        self.idle_sleep_dt = 0.1
        self.wait_sleep_dt = 0.02

        # Initialize blob finder
        self.blobFinder = BlobFinder()
        self.blobFinder.threshold = 200 
        self.blobFinder.filter_by_area = False
        self.blobFinder.min_area = 0
        self.blobFinder.max_area = 200
       
        rospy.init_node('homography_calibrator')
        self.image_sub = rospy.Subscriber(self.topic,Image,self.image_callback)
        self.image_blobs_pub = rospy.Publisher('image_blobs', Image)
        self.image_calib_pub = rospy.Publisher('image_calibration', Image)

    def image_callback(self,data):
        # Find blobs
        with self.lock:
            self.blobs_list, blobs_rosimage = self.blobFinder.findBlobs(data)
            if self.image_wait_cnt < self.image_wait_number:
                self.image_wait_cnt += 1

        self.image_blobs_pub.publish(blobs_rosimage)

        # Render calibration data image
        cv_image = self.bridge.imgmsg_to_cv(data,desired_encoding="passthrough")
        ipl_image = cv.GetImage(cv_image)
        calib_image = cv.CreateImage(cv.GetSize(ipl_image), cv.IPL_DEPTH_8U, 3)
        cv.CvtColor(ipl_image,calib_image,cv.CV_GRAY2BGR)

        for x,y in self.image_points:
            cv.Circle(calib_image, (int(x),int(y)),3, (0,0,255))

        calib_rosimage = self.bridge.cv_to_imgmsg(calib_image,encoding="passthrough")
        self.image_calib_pub.publish(calib_rosimage)


    def wait_for_images(self):
        """
        Wait for 
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
        if self.led_m < self.led_m_max-1 or self.led_n < self.led_n_max-1:
            if self.led_n < self.led_n_max-1:
                self.led_n += 1
            else:
                self.led_n  = 0
                self.led_m += 1
        else:
            # Need to check that we got enough done otherwise return to idle.
            self.state = 'done'

    def run(self):
        while not rospy.is_shutdown():

            if self.state == 'calibrating':
                mct_active_target.set_led(self.led_n, self.led_m, self.led_power)
                self.wait_for_images()
                if len(self.blobs_list) == 1:
                    blob = self.blobs_list[0]
                    image_x = blob['centroid_x']
                    image_y = blob['centroid_y']
                    self.image_points.append((image_x, image_y))
                    world_x = self.led_n*self.led_space_x
                    world_y = self.led_m*self.led_space_y
                    self.world_points.append((world_x, world_y))
                    print(len(self.image_points))

                self.increment_led()
                if self.state == 'done':


                    # Find the homography transformation
                    image_points = numpy.array(self.image_points)
                    world_points = numpy.array(self.world_points)
                    self.h_matrix, mask = cv2.findHomography(image_points, world_points, cv.CV_RANSAC)

                    # Compute the mean reprojection error
                    image_points2 = numpy.ones((image_points.shape[0],3))
                    image_points2[:,:2] = image_points
                    world_points2 = numpy.ones((world_points.shape[0],3))
                    world_points2[:,:2] = world_points
                    h_matrix_t = self.h_matrix.transpose()
                    world_points2_pred = numpy.dot(image_points2, h_matrix_t)
                    error = (world_points2[:,:2] - world_points2_pred[:,:2])**2
                    error = error.sum(axis=1)
                    error = numpy.sqrt(error)
                    error = error.mean()
                    print('reprojection error:', error)
                    print()

                    

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
