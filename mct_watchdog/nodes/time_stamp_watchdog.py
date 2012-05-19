#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('mct_watchdog')
import rospy
import threading
import functools
import numpy
import mct_introspection
import yaml

import cv
import Image as PILImage
import ImageDraw as PILImageDraw
import ImageFont as PILImageFont

from cv_bridge.cv_bridge import CvBridge 
from mct_utilities import file_tools

# Services
from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse

# Messages
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from mct_msg_and_srv.msg import TimeStampWatchDog

class TimeStampWatchdog(object):

    """
    Watchdog for hardware triggered cameras. Checks for correct sequence order
    and maximum allowed time stamp error.
    """
    def __init__(self, frame_rate, max_allowed_error=1.0e-3, max_seq_age=200):

        self.frame_rate = frame_rate

        self.lock = threading.Lock()
        self.max_allowed_error = max_allowed_error
        self.max_seq_age = max_seq_age

        self.seq_to_camera_stamps = {}
        self.stamp_last = {}

        self.max_error_by_camera = {}
        self.max_error = 0.0
        self.camera_fail = ''

        self.most_recent_seq = None
        self.last_checked_seq = None

        self.ok = True
        self.seq_fail = 0 
        self.error_msg = ''

        self.bridge = CvBridge()
        self.info_image_size = (400,90)
        self.font = PILImageFont.truetype("/usr/share/fonts/truetype/ubuntu-font-family/Ubuntu-B.ttf", 16)

        self.ready = False
        rospy.init_node('time_stamp_watchdog')

        # Subscribe to camera info topics
        self.wait_for_camera_info_topics()
        camera_info_topics = mct_introspection.find_camera_info_topics()
        self.camera_info_sub = {}
        for topic in camera_info_topics:
            camera = get_camera_from_topic(topic)
            handler = functools.partial(self.camera_info_handler, camera)
            self.camera_info_sub[camera] = rospy.Subscriber(topic, CameraInfo, handler)
        self.number_of_cameras = len(camera_info_topics)

        # Create reset service
        self.reset_srv = rospy.Service('time_stamp_watchdog_reset', Empty, self.reset_handler)

        # Create time stamp watchdog publication
        self.watchdog_pub = rospy.Publisher('time_stamp_watchdog', TimeStampWatchDog)

        # Create watchdog info image
        self.image_watchdog_pub = rospy.Publisher('image_time_stamp_watchdog', Image)

        self.ready = True

    def wait_for_camera_info_topics(self):
        """
        Wait until the number of camera info topics is equal to the number of
        cameras in the current camera assignment.
        """
        camera_assignment = file_tools.read_camera_assignment()
        number_of_cameras = len(camera_assignment)
        while 1:
            camera_info_topics = mct_introspection.find_camera_info_topics()
            if len(camera_info_topics) == number_of_cameras:
                break
            rospy.sleep(0.25)


    def reset_handler(self,req):
        """
        Handles reset service
        """
        with self.lock:
            self.seq_to_camera_stamps = {}
            self.stamp_last = {}
            self.max_error_by_camera = {}
            self.max_error = 0.0
            self.most_recent_seq = None
            self.last_checked_seq = None
            self.ok = True
            self.seq_fail = 0 
            self.error_msg = ''
            self.camera_fail = ''
        return EmptyResponse()

    def camera_info_handler(self, camera, data):
        """
        Handler for camera info messages. Stores the time stamps based on
        seqeunce number and camera.
        """
        if not self.ready:
            return

        with self.lock:
            seq = data.header.seq
            stamp = data.header.stamp
            try:
                self.seq_to_camera_stamps[seq][camera] = stamp
            except KeyError:
                self.seq_to_camera_stamps[seq] = {camera: stamp}
            self.most_recent_seq = seq

    def check_camera_stamps(self, seq, camera_stamps):
        """
        Checks the time stamps for a given sequence number. Examines the
        sequence number to ensure it is the next expected and exaimes the
        spread of the time stamps to ensure that is less than the allowed
        maximum. The results are published on the watchdog topic.
        """
        cur_seq_ok = True
        error_list = []
        # Check for correct sequence order
        if self.last_checked_seq is not None:
            if seq != self.last_checked_seq+1:
                cur_seq_ok = False
                self.seq_fail = seq
                error_list.append('sequence out of order')
        self.last_checked_seq = seq

        # Get the max and min time stamps for cameras on the same machine
        stamp_delta = {}
        stamp_error = {}

        for camera, stamp in camera_stamps.iteritems():
            # Get differenece between current and last time stamps and compute the stamp error
            try:
                stamp_delta[camera] = stamp.to_sec() - self.stamp_last[camera].to_sec()
            except KeyError:
                stamp_delta[camera] = 1.0/self.frame_rate 
            stamp_error[camera] = abs(1.0/self.frame_rate - stamp_delta[camera])

            # Find the maximum error so far for each camera
            try:
                self.max_error_by_camera[camera] = max([self.max_error_by_camera[camera], stamp_error[camera]])
            except KeyError:
                self.max_error_by_camera[camera] = stamp_error[camera]

            self.stamp_last[camera] = stamp

        # Compute the maximum error so far for all cameras
        self.max_error = max([err for err in self.max_error_by_camera.values()])
        if self.max_error > self.max_allowed_error:
            camera_fail_list = [cam for cam, err in self.max_error_by_camera.items() if err > self.max_allowed_error]
            self.camera_fail = str(camera_fail_list)
            cur_seq_ok = False
            error_list.append('time stamp outside of expected range')


        # If this is the first seq with an error set the error message
        if self.ok and not cur_seq_ok:
            self.ok = False
            self.seq_fail = seq
            self.error_msg = ', '.join(error_list)

        # Publish watchdog message
        watchdog_msg = TimeStampWatchDog()
        watchdog_msg.seq = seq
        watchdog_msg.ok = self.ok 
        watchdog_msg.frame_rate = self.frame_rate
        watchdog_msg.max_allowed_error = self.max_allowed_error
        watchdog_msg.max_error = self.max_error
        watchdog_msg.seq_fail = self.seq_fail
        watchdog_msg.error_msg = self.error_msg
        watchdog_msg.camera_fail = self.camera_fail
        self.watchdog_pub.publish(watchdog_msg)

        # Publish watchdog image
        pil_info_image = PILImage.new('RGB', self.info_image_size,(255,255,255))
        draw = PILImageDraw.Draw(pil_info_image)
        info_items = [ ('ok', 'ok', ''), ('seq','seq',''), ('max_error', 'max error', 's')]
        text_x, text_y, step_y = 10, 10, 20
        for i, item in enumerate(info_items):
            name, label, units = item
            value = getattr(watchdog_msg,name)
            label_text = '{0}:'.format(label)
            if type(value) == float:
                value_text = '{0:<1.6f}'.format(value)
            else:
                value_text = '{0}'.format(value)
            units_text = '{0}'.format(units)
            draw.text( (text_x,text_y+step_y*i), label_text, font=self.font, fill=(0,0,0))
            draw.text( (text_x+100,text_y+step_y*i), value_text, font=self.font, fill=(0,0,0))
            draw.text( (text_x+180,text_y+step_y*i), units_text, font=self.font, fill=(0,0,0))

        if self.error_msg:
            error_text = 'Error: {0}'.format(self.error_msg)
        else:
            error_text = ''

        draw.text(
                (text_x, text_y+len(info_items)*step_y),
                error_text,
                font=self.font,
                fill=(255,0,0),
                )

        cv_info_image = cv.CreateImageHeader(pil_info_image.size, cv.IPL_DEPTH_8U, 3)
        cv.SetData(cv_info_image, pil_info_image.tostring())

        # Convert to a rosimage and publish
        info_rosimage = self.bridge.cv_to_imgmsg(cv_info_image,'rgb8')
        self.image_watchdog_pub.publish(info_rosimage)


    def process_camera_stamps(self):
        """
        Processes the camera time stamps. First, looks to see if all the images
        have been recieved for a given sequence number. Second, check if any of
        the sequences are older than the maximum allowed age and if so throws
        them away. 
        """
        for seq, camera_stamps in sorted(self.seq_to_camera_stamps.items()):
            if len(camera_stamps) == self.number_of_cameras:
                self.check_camera_stamps(seq, camera_stamps)
                del self.seq_to_camera_stamps[seq]
            else:
                if self.most_recent_seq is not None:
                    seq_age = self.most_recent_seq - seq
                    if seq_age > self.max_seq_age:
                        del self.seq_to_camera_stamps[seq]

    def run(self):
        """
        Node, main loop.  While the node
        """
        while not rospy.is_shutdown():
            with self.lock:
                self.process_camera_stamps()


# Utility functions
# ----------------------------------------------------------------------------

def get_camera_from_topic(topic):
    camera = topic.split('/')[2]
    return camera

def get_machine_from_topic(topic):
    machine = topic.split('/')[1]
    return machine

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    # Temporary get tracking 2d frame rate. When/if we have more tracking modes
    # we will probably want to pass this to the node at launch.
    frame_rate_dict = file_tools.read_frame_rates()
    frame_rate = frame_rate_dict['tracking_2d']

    node = TimeStampWatchdog(frame_rate)
    node.run()
