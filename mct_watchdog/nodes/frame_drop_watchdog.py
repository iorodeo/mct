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

# Messages and Services
from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse
from mct_msg_and_srv.msg import FramesDropped
from sensor_msgs.msg import Image

class FrameDropWatchdog(object):

    """
    Frame drop watchdog monitors the number of frames dropped by the system.     
    """
    def __init__(self ):

        rospy.init_node('frame_drop_watchdog')
        self.lock = threading.Lock()
        self.ready = False
        self.frames_dropped = {}

        camera_assignment = file_tools.read_camera_assignment()
        self.number_of_cameras = len(camera_assignment)

        self.bridge = CvBridge()
        self.info_image_size = (400,90)
        self.font = PILImageFont.truetype("/usr/share/fonts/truetype/ubuntu-font-family/Ubuntu-B.ttf", 16)

        # Subscribe to camera info topics
        self.frames_dropped_sub = {}
        frames_dropped_topics = self.wait_for_topics()
        for topic in frames_dropped_topics:
            camera = get_camera_from_topic(topic)
            handler = functools.partial(self.frames_dropped_handler, camera)
            self.frames_dropped_sub[camera] = rospy.Subscriber(topic, FramesDropped, handler)


        # Setup total frames dropped service
        self.total_dropped_pub = rospy.Publisher('total_frames_dropped', FramesDropped)

        # Setup reset service
        self.reset_srv = rospy.Service('frame_drop_watchdog_reset', Empty, self.reset_handler)

        # Create watchdog info image
        self.image_watchdog_pub = rospy.Publisher('image_frame_drop_watchdog', Image)

        self.ready = True


    def wait_for_topics(self):
        """
        Wait for the frames_dropped topics to be published.
        """
        while 1:
            frames_dropped_topics = mct_introspection.find_topics_w_ending('frames_dropped')
            if len(frames_dropped_topics) == self.number_of_cameras:
                break
            rospy.sleep(0.25)
        return frames_dropped_topics

    def reset_handler(self,req):
        """
        Handler for the nodes reset service - empties the frames_dropped buffer.
        """
        with self.lock:
            self.frames_dropped = {}
        return EmptyResponse()

    def frames_dropped_handler(self, camera, data):
        if not self.ready:
            return
        with self.lock:
            try:
                self.frames_dropped[data.seq][camera] = data.frames_dropped
            except KeyError:
                self.frames_dropped[data.seq] = {camera:data.frames_dropped}

    def publish_watchdog_image(self, seq, total_frames_dropped, cameras_w_drops): 
        """
        Publish image for GUI w/ seq #, total frames dropped, other info?
        """
        pil_info_image = PILImage.new('RGB', self.info_image_size,(255,255,255))
        draw = PILImageDraw.Draw(pil_info_image)
        info_items = [ 
                ('seq',     seq), 
                ('dropped', total_frames_dropped), 
                ('cameras', cameras_w_drops), 
                ]
        text_x, text_y, step_y = 10, 10, 20
        for i, item in enumerate(info_items):
            label, value = item
            label_text = '{0}:'.format(label)
            if type(value) == float:
                value_text = '{0:<1.6f}'.format(value)
            else:
                value_text = '{0}'.format(value)
            draw.text( (text_x,text_y+step_y*i), label_text, font=self.font, fill=(0,0,0))
            draw.text( (text_x+100,text_y+step_y*i), value_text, font=self.font, fill=(0,0,0))

        cv_info_image = cv.CreateImageHeader(pil_info_image.size, cv.IPL_DEPTH_8U, 3)
        cv.SetData(cv_info_image, pil_info_image.tostring())

        # Convert to a rosimage and publish
        info_rosimage = self.bridge.cv_to_imgmsg(cv_info_image,'rgb8')
        self.image_watchdog_pub.publish(info_rosimage)

    def run(self):
        """
        Node, main loop.  While the node
        """
        while not rospy.is_shutdown():
            with self.lock:
                for seq, data in sorted(self.frames_dropped.items()):
                    if len(data) == self.number_of_cameras:
                        total_frames_dropped = sum(data.values())
                        self.total_dropped_pub.publish(seq,total_frames_dropped)
                        cameras_w_drops = [c for c, n in data.iteritems() if n > 0]
                        cameras_w_drops = [int(c.split('_')[1]) for c in cameras_w_drops]
                        del self.frames_dropped[seq] 
                        self.publish_watchdog_image(seq, total_frames_dropped, cameras_w_drops)



# Utility functions
# ----------------------------------------------------------------------------
def get_camera_from_topic(topic):
    camera = topic.split('/')[2]
    return camera


# -----------------------------------------------------------------------------
if __name__ == '__main__':

    node = FrameDropWatchdog()
    node.run()
