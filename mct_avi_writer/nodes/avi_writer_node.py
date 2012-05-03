#!/usr/bin/env python
import roslib
roslib.load_manifest('mct_avi_writer')
import rospy
import sys
import os
import os.path
import cv
import threading
import math

from mct_utilities import file_tools

# Messages
from sensor_msgs.msg import Image
from cv_bridge.cv_bridge import CvBridge 
from cv_bridge.cv_bridge import CvBridgeError
from mct_msg_and_srv.msg import RecordingProgressMsg

# Services
from mct_msg_and_srv.srv import RecordingCmd
from mct_msg_and_srv.srv import RecordingCmdResponse

class AVI_Writer(object):

    def __init__(self,topic,frame_rate):

        self.topic = topic
        self.frame_rate = frame_rate
        self.start_t = 0.0
        self.current_t = 0.0
        self.progress_t = 0.0 
        self.frame_count = 0
        self.recording_message = 'stopped'

        self.writer = None 
        self.done = True 
        self.cv_image_size = None
        self.filename = os.path.join(os.environ['HOME'],'default.avi')

        self.lock = threading.Lock()
        self.bridge = CvBridge()
        rospy.init_node('avi_writer')

        # Set up publications
        self.progress_msg = RecordingProgressMsg()
        self.progress_pub = rospy.Publisher('progress',RecordingProgressMsg)

        # Subscribe to messages
        self.image_sub = rospy.Subscriber(self.topic,Image,self.image_handler)

        # Set up services
        self.recording_srv = rospy.Service(
                'recording_cmd', 
                RecordingCmd, 
                self.handle_recording_cmd
                )

    def run(self):
        rospy.spin()

    def handle_recording_cmd(self,req):
        """
        Handles avi recording commands - starts and stops avi recording.
        """
        with self.lock:

            if self.cv_image_size is None:
                # If we don't have and image yet - we can't get started, return fail.
                return RecordingCmdResponse(False)

            self.filename = req.filename
            command = req.command.lower()

            if command == 'start':
                # Get start time and create video writer
                self.writer = cv.CreateVideoWriter(
                        self.filename,
                        cv.CV_FOURCC('D','I','V','X'),
                        self.frame_rate,
                        self.cv_image_size,
                        ) 

                if self.writer is None:
                    response = False
                else:
                    response = True
                    self.start_t = rospy.get_time()
                    self.frame_count = 0
                    self.done = False
                    self.recording_message = 'recording'

            elif command == 'stop':
                self.done = True
                del self.writer
                self.writer = None
                self.recording_message = 'stopped'
                response = True

            return RecordingCmdResponse(response)

    def image_handler(self,data): 
        """
        Writes frames to avi file.
        """
        self.current_t = rospy.get_time()

        # Convert to opencv image and then to ipl_image
        cv_image = self.bridge.imgmsg_to_cv(data,'bgr8')
        ipl_image = cv.GetImage(cv_image)

        with self.lock:
            if self.cv_image_size == None:
                self.cv_image_size = cv.GetSize(cv_image)

        if not self.done:

            # Write video frame
            cv.WriteFrame(self.writer,ipl_image)

            # Update times and frame count - these are used elsewhere so we 
            # need the lock
            with self.lock:
                self.frame_count += 1
                self.progress_t = self.current_t - self.start_t

        # Publish progress message
        with self.lock:
            self.progress_msg.frame_count = self.frame_count
            self.progress_msg.progress_t = self.progress_t
            self.progress_msg.recording_message = self.recording_message
        self.progress_pub.publish(self.progress_msg)

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    topic = sys.argv[1]
    frame_rate = float(sys.argv[2])

    node = AVI_Writer(topic,frame_rate)
    node.run()


