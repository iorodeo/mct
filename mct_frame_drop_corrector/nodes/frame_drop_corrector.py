#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('mct_frame_drop_corrector')
import rospy
import sys
import threading

import cv
from cv_bridge.cv_bridge import CvBridge 
from mct_utilities import file_tools

# Messages
from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from mct_msg_and_srv.msg import SeqAndImage
from mct_msg_and_srv.srv import FrameDropInfo
from mct_msg_and_srv.srv import FrameDropInfoResponse

SEC_TO_NSEC = int(1e9)

class Frame_Drop_Corrector(object):

    """
    Frame drop corrector node. Subscribes to the given image topic and detects
    dropped frames. Dummy frames are inserted for any frames which are dropped.
    """
    def __init__(self, topic, framerate, max_stamp_age=1.5, publish_image_corr=True):
        self.ready = False
        self.topic = topic
        self.framerate = framerate
        self.publish_image_corr = publish_image_corr
        self.lock = threading.Lock() 
        self.camera_info_topic = get_camera_info_from_image_topic(self.topic)
        self.max_stamp_age = max_stamp_age
        self.bridge = CvBridge()
        self.last_pub_stamp = None
        self.latest_stamp = None
        self.dummy_image = None
        self.seq_offset = 0
        self.frame_drop_list = []

        # Data dictionaries for synchronizing tracking data with image seq number
        self.stamp_to_image = {}
        self.stamp_to_seq = {}
        self.seq_to_stamp_and_image = {}

        rospy.init_node('frame_drop_corrector')

        # Subscribe to image and camera info topics
        self.image_sub = rospy.Subscriber(
                self.topic, 
                Image, 
                self.image_callback
                )
        self.info_sub = rospy.Subscriber(
                self.camera_info_topic, 
                CameraInfo, 
                self.camera_info_callback
                )

        # Set up image publisher
        topic_split = self.topic.split('/')
        seq_and_image_topic = topic_split[:-1]
        seq_and_image_topic.append('seq_and_image_corr')
        seq_and_image_topic = '/'.join(seq_and_image_topic)
        self.seq_and_image_repub = rospy.Publisher(seq_and_image_topic,SeqAndImage)
        if self.publish_image_corr:
            image_topic = topic_split[:-1]
            image_topic.append('image_corr')
            image_topic = '/'.join(image_topic)
            self.image_repub = rospy.Publisher(image_topic, Image)

        # Set up frame drop info service - returns list of seq numbers
        # corresponding to the dropped frames for image stream.
        frame_drop_srv_name = topic_split[:-1]
        frame_drop_srv_name.append('frame_drop_info')
        frame_drop_srv_name = '/'.join(frame_drop_srv_name)
        self.frame_drop_srv = rospy.Service(
                frame_drop_srv_name, 
                FrameDropInfo, 
                self.handle_frame_drop_srv)

        # Setup reset service - needs to be called anytime the camera trigger is 
        # stopped. It resets the last_pub_stamp to none.
        reset_srv_name = topic_split[:-1]
        reset_srv_name.append('frame_drop_reset')
        reset_srv_name = '/'.join(reset_srv_name)
        self.reset_srv = rospy.Service(reset_srv_name,Empty,self.handle_reset_srv)
        self.ready = True

    def handle_reset_srv(self,req):
        """
        Resets the frame not detection node. This prevents the addition of a huge
        number of 'false' dropped frames when the system is restarted.
        """
        with self.lock:
            self.last_pub_stamp = None
            self.frame_drop_list = []
        return EmptyResponse()

    def handle_frame_drop_srv(self,req):
        """
        Returns list of sequences numbers for all dropped frames.
        """
        with self.lock:
            frame_drop_list = list(self.frame_drop_list)
        return FrameDropInfoResponse(frame_drop_list)

    def camera_info_callback(self,data):
        """
        Callback for camera info topic subscription - used to associate stamp
        with the image seq number.
        """
        if not self.ready:
            return
        stamp_tuple = data.header.stamp.secs, data.header.stamp.nsecs
        with self.lock:
            self.latest_stamp = stamp_tuple
            self.stamp_to_seq[stamp_tuple] = data.header.seq


    def image_callback(self,data):
        """
        Callback for image topic subscription - used to associate the stamp
        tuple with the imcomming image.
        """
        if not self.ready:
            return 
        stamp_tuple = data.header.stamp.secs, data.header.stamp.nsecs
        with self.lock:
            self.stamp_to_image[stamp_tuple] = data

            if self.dummy_image is None:
                # Create dummy image for use when a dropped frame occurs.
                cv_image = self.bridge.imgmsg_to_cv(data,desired_encoding="passthrough")
                raw_image = cv.GetImage(cv_image)
                dummy_cv_image = cv.CreateImage(cv.GetSize(raw_image),raw_image.depth, raw_image.channels)
                cv.Zero(dummy_cv_image)
                self.dummy_image = self.bridge.cv_to_imgmsg(dummy_cv_image,encoding="passthrough")


    def associate_image_w_seq(self): 
        """
         Associate iamges  with image seq numbers
        """
        with self.lock:
            for stamp, image in self.stamp_to_image.items():
                try:
                    seq = self.stamp_to_seq[stamp]
                    seq_found = True
                except KeyError:
                    seq_found = False

                if seq_found:
                    self.seq_to_stamp_and_image[seq] = stamp, image 
                    try:
                        del self.stamp_to_image[stamp]
                    except KeyError:
                        pass
                    try:
                        del self.stamp_to_seq[stamp]
                    except KeyError:
                        pass
                   
                else:
                    # Throw away data greater than the maximum allowed age
                    if self.latest_stamp is not None:
                        latest_stamp_secs = stamp_tuple_to_secs(self.latest_stamp)
                        stamp_secs = stamp_tuple_to_secs(stamp)
                        stamp_age = latest_stamp_secs - stamp_secs
                        if stamp_age > self.max_stamp_age:
                            try:
                                del self.stamp_to_image[stamp]
                            except KeyError:
                                pass


    def republish_seq_and_image(self): 
        """
        Republished the sequence numbers and images with black frames inserted for
        any dropped frames which are discovered by looking at the time stamps.
        """
        for seq, stamp_and_image in sorted(self.seq_to_stamp_and_image.items()):
            stamp_tuple, image = stamp_and_image 
            
            if self.last_pub_stamp is not None:
                dt = stamp_dt_secs(stamp_tuple, self.last_pub_stamp)
                framegap = int(round(dt*self.framerate))

                for i in range(framegap-1):
                    # Note, framegap > 1 implies that we have drop frames. 
                    # Publish dummies frames until we are caught up.
                    dummy_seq = seq + self.seq_offset
                    dummy_stamp_tuple = incr_stamp_tuple(self.last_pub_stamp, (i+1)/self.framerate)
                    dummy_image = self.dummy_image
                    dummy_image.header.seq = dummy_seq 
                    dummy_image.header.stamp = rospy.Time(*dummy_stamp_tuple)
                    self.seq_and_image_repub.publish(dummy_seq, dummy_image)
                    if self.publish_image_corr:
                        self.image_repub.publish(dummy_image)
                    self.seq_offset += 1
                    self.frame_drop_list.append(dummy_seq)

            # Publish new frame with corrected sequence number - to account for dropped frames
            corrected_seq = seq + self.seq_offset
            image.header.seq = corrected_seq 
            image.header.stamp = rospy.Time(*stamp_tuple)
            self.seq_and_image_repub.publish(corrected_seq, image)
            if self.publish_image_corr:
                self.image_repub.publish(image)
            self.last_pub_stamp = stamp_tuple
            del self.seq_to_stamp_and_image[seq]


    def run(self):
        """
        Main loop. Associates images and time stamps w/ image sequence numbers.
        Examines time interval between frames to detect dropped frames.
        Re-publishes sequences and images as combined SeqAndImage topic. When a
        dropped frame is detected a blank "dummy" frame is inserted in its place.
        """
        while not rospy.is_shutdown():
            self.associate_image_w_seq()
            self.republish_seq_and_image()


def get_camera_info_from_image_topic(topic):
    """
    Returns the camera info topic given an image topic from that camera
    """
    topic_split = topic.split('/')
    info_topic = topic_split[:-1]
    info_topic.append('camera_info')
    info_topic = '/'.join(info_topic)
    return info_topic

def stamp_tuple_to_secs(stamp):
    """
    Converts a stamp tuple (secs,nsecs) to seconds.
    """
    return stamp[0] + stamp[1]*1.0e-9

def stamp_dt_secs(stamp1, stamp0):
    stamp1_secs = stamp_tuple_to_secs(stamp1)
    stamp0_secs = stamp_tuple_to_secs(stamp0)
    return stamp1_secs - stamp0_secs

def incr_stamp_tuple(stamp, dt):
    dt_nsec = int(dt*SEC_TO_NSEC)
    stamp_sec, stamp_nsec = stamp
    temp_nsec = stamp_nsec + dt_nsec
    stamp_nsec = temp_nsec%int(SEC_TO_NSEC)
    stamp_sec += temp_nsec/int(SEC_TO_NSEC)
    return stamp_sec, stamp_nsec

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    topic = sys.argv[1]
    framerate = float(sys.argv[2])
    node = Frame_Drop_Corrector(topic,framerate)
    node.run()




